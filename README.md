# PointWeaver

Point-to-point path following for **CTRE Phoenix 6 swerve** drivetrains (FRC, **WPILib 2027 alpha-7**).
Draw paths in the PointWeaver app, drop them in your deploy folder, follow them with a few lines of code.
No splines: the robot drives straight at each point, and the library works out how fast it can go.

> For WPILib 2026 robots, use the legacy `2026` branch and its vendordep.

- Speed comes from a braking curve (it can always still stop at the point) and a final-approach P loop, with one acceleration limit for starting, stopping and corners.
- Heading uses CTRE's `FieldCentricFacingAngle`, optionally interpolated smoothly between heading points.
- Per-point overrides for speed, acceleration, early exit and timeout. Named events and optional odometry reset.
- Works with OpModes, plain `TimedRobot`, and command-based (Commands V2).

## Install

1. Install CTRE Phoenix 6 for 2027 (`26.70.0-alpha-2` or newer) if you haven't.
2. In VS Code: **WPILib: Manage Vendor Libraries → Install new library (online)** and paste:
   ```
   https://tasrobotics.github.io/pointweaver-lib/2027/PointWeaver.json
   ```
3. Using `PointWeaverCommands`? Also install the **Commands V2** vendordep that ships with WPILib.

## Files

The PointWeaver app writes both of these into your robot project:

```
src/main/deploy/pointweaver/
  robot.json      your robot's speed, acceleration and tuning (shared with the app's preview)
  paths/*.json    one file per path
```

## Use it

```java
public class Robot extends OpModeRobot {
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Robot() {
        PointWeaver.configure(drivetrain); // reads deploy/pointweaver/robot.json
        PointWeaver.registerEvent("intake", () -> intake.start());
    }
}

@Autonomous(name = "Test Path")
public class TestPathAuto implements OpMode {
    private final PathFollower follower = PointWeaver.follower("testpath");

    @Override public void start() { follower.start(); }
    @Override public void periodic() { follower.update(); } // until follower.isFinished()
    @Override public void end() { follower.stop(); }
}
```

`TimedRobot` works the same way: `start()` in `autonomousInit`, `update()` in `autonomousPeriodic`, `stop()` in `autonomousExit`.

**Command-based** (Commands V2 vendordep):

```java
PointWeaverCommands.registerEvent("intake", intake.runCommand()); // must not require the drivetrain
Command auto = PointWeaverCommands.followPath("testpath");
```

If a path file is missing or broken, the error goes to the Driver Station and the robot stays still. It doesn't crash.

## robot.json

Edit it in the app's Robot panel. The app can also fill speed, spin rate and size from your `TunerConstants.java`. Every field is optional except `maxSpeed`.

| Field | Default | What it does |
|---|---|---|
| `maxSpeed` | required | Top speed (m/s), e.g. `kSpeedAt12Volts`. Paths can only go slower. |
| `maxAccel` | 3.0 | Acceleration limit (m/s²) for speeding up, braking and corners. Raise it until the wheels slip, then back off. |
| `translationPID` | [5, 0, 0] | Final-approach gain on distance to the point. Only takes over in the last ~20 cm. |
| `headingPID` | [2, 0, 0] | CTRE heading controller gains. |
| `positionTolerance` | 0.05 | A point counts as reached within this distance of it (m). |
| `headingTolerance` | 2 | Heading points also need this (deg). |
| `maxAngularRate`, `robotLength`, `robotWidth` | | Used by the app's preview (and heading planning, coming next). |

Rather set things in code? `PointWeaverConfig.fromDeploy().headingPID(3, 0, 0).build()` overrides single values, and `PointWeaverConfig.builder(maxSpeed)` skips the file entirely.

## How speed is chosen

Each loop, toward the current point at distance `d`:

1. `braking = sqrt(2 · 0.9·maxAccel · d)` is the fastest speed that can still stop at the point. The 0.9 leaves room for steering.
2. `planned = min(braking, translationPID(d))`, clamped to the point's min/max speed.
3. The commanded velocity moves toward `planned` in the point's direction by at most `maxAccel · 0.02` per loop, as a vector. That one limit covers starting, stopping and turning, so corners round off on their own.

## Path rules

The JSON format and every field are described in [docs/path-format.md](docs/path-format.md). The short version:

- Points are driven to in order. A point **without** a heading is translation-only.
- `interpolate` on a heading point means "turn smoothly into this heading" across the stretch since the previous heading point. Off means aim for it right away, as soon as the previous heading point is done. Turning it on for the whole path turns it on for every heading point.
- `earlyExitRange` moves on early, which is good for flowing through points. The **last point always arrives**: early exit and min speed don't apply to it.
- Translation-only points never wait on heading.
- Events fire at a fraction of the way to the next point. Anything skipped by early exit still fires when the point finishes.
- `resetPose: true` resets odometry to point 0 when the path starts.

## Telemetry

Under `/PointWeaver/` on NetworkTables:
- `pose`, `target`, `path`: Pose2d structs, so AdvantageScope and Elastic can draw them on a field.
- `speed` vs `plannedSpeed`: compare them while tuning `maxAccel`.
- `activePath`, `running`, `targetIndex`, `distance`, `desiredHeadingDeg`, `vx`, `vy`, `lastEvent`.

## Developing

```
./gradlew build      # compile + tests; needs JDK 25 (e.g. the WPILib 2027 JDK), nothing else installed
```

- `pointweaver.core` is plain Java (no WPILib, no CTRE). All the following logic lives there and is unit tested.
- `FollowerCoreSimTest` pins the follower to the PointWeaver app's ghost (exact timings on three paths), so the app preview and the robot can't drift apart.
- The rest of `pointweaver` is thin glue: CTRE drivetrain, NetworkTables, deploy folder. `pointweaver.command` is the optional Commands v2 adapter.
- **Testing in a robot project without publishing:** put this repo next to it and add `includeBuild('../pointweaver-lib')` to its `settings.gradle`. Gradle uses the local source instead of the released jar.
- **Releasing:** push a tag like `v2027.0.0` from `main`. GitHub Actions publishes the jar and the vendordep to GitHub Pages under `/2027/`, leaving the 2026 vendordep alone. Enable Pages on the `gh-pages` branch once.
- **Coming from 2026:** [docs/migrating-to-2027.md](docs/migrating-to-2027.md) lists what changed.

MIT licensed.
