# Path file format

One JSON file per path, in `src/main/deploy/pointweaver/paths/<name>.json`. Robot-wide settings live next to it in
`robot.json` (see the README). Coordinates are field meters from the
blue alliance origin, and angles are degrees CCW from +x. The PointWeaver app writes this format. Unknown fields
are ignored, so newer files load on older versions.

```json
{
  "resetPose": true,
  "params": { "maxSpeed": 3.0, "earlyExitRange": 0.5, "interpolate": true, "timeout": 8.0 },
  "points": [
    { "x": 5.0, "y": 3.0 },
    { "x": 4.7, "y": 4.7, "theta": -135, "params": { "minSpeed": 2.0 },
      "events": [{ "name": "intake", "fraction": 0.5 }] },
    { "x": 3.0, "y": 5.0, "theta": 180 }
  ]
}
```

## Top level

| Field | Type | Meaning |
|---|---|---|
| `points` | list | Points, driven to in order. |
| `params` | object or null | Defaults for every point (see below). |
| `resetPose` | bool | Reset odometry to point 0 when the path starts. If point 0 has no `theta`, only x/y are reset. |

## Point

| Field | Type | Meaning |
|---|---|---|
| `x`, `y` | number | Required. |
| `theta` | number or null | Heading. Null/missing = translation-only point. |
| `params` | object or null | Overrides the path `params` for this point. |
| `events` | list | `{ "name": string, "fraction": 0..1 }` |

## Params (path-wide or per point)

Each field can be missing or null, which means "inherit". A point's value wins over the path's, except
`interpolate`: true on the path turns it on for every point.

| Field | Default | Meaning |
|---|---|---|
| `maxSpeed` | robot max | Top speed toward this point (m/s). |
| `maxAccel` | robot max | Acceleration limit toward this point (m/s²), e.g. gentler while holding a game piece. Capped by `robot.json`. |
| `minSpeed` | 0 | Never slower than this toward this point. **Ignored on the last point.** |
| `earlyExitRange` | off | Move on once this close (m). **Ignored on the last point.** |
| `timeout` | none | Move on after this many seconds no matter what. |
| `interpolate` | false | Only matters on heading points, see below. True on the path = true on every point (a point can't turn it back off); otherwise each point decides. |
| `turnDirection` | `"auto"` | **Points only** (ignored on the path). Which way an interpolated turn into this heading goes round: `"cw"`, `"ccw"`, or `"auto"` (the shorter way). See below. |

## Speed

Toward each point the robot uses the slower of a braking curve (`sqrt(2·0.9·a·d)`, so it can always still stop at
the point) and the final-approach P loop, clamped to `minSpeed`/`maxSpeed`. The commanded velocity then changes by
at most `a·0.02` m/s per loop as a vector, where `a` is the point's `maxAccel`. Starting, stopping and corners all
share that limit. Early exit hands off to the next point while still moving, so the robot flows through.

## Headings

The path splits into stretches from one heading point **A** to the next heading point **B**, with
translation-only points in between. Before the first heading point, A is the robot's heading when the path started.

- **`interpolate` true on B (or on the path):** the target heading eases from A to B (smoothstep) over the whole stretch, measured
  along the path. No snapping.
- **`interpolate` false on B:** aim for B as soon as the robot leaves A, turning as fast as the heading controller allows.
- **`turnDirection` on B** picks the way round for an interpolated turn: `"cw"` (heading goes down) or `"ccw"` (heading
  goes up, the WPILib direction), even when the other way is shorter, so 0° → 90° with `"cw"` turns 270°. `"auto"` or
  unset takes the shorter way. Already facing B means no turn either way. Without `interpolate` it has no effect: the
  heading controller always takes the shorter way to a fixed target.

## When a point is done

Checked every loop, in this order:
1. its `timeout` has passed, or
2. (not the last point) it's within `earlyExitRange`, or
3. it's within the position tolerance of it (straight-line distance) **and**, for heading points only, within the heading tolerance.

Translation-only points never wait for heading, because the robot may be mid-turn on purpose.

## Events

An event on point *i* fires once the robot's projection onto the segment *i → i+1* reaches `fraction`.
Events skipped by early exit fire when the point finishes. Events on the last point fire when the path finishes.
Register what they do with `PointWeaver.registerEvent(name, ...)`. An unknown name logs one warning.
