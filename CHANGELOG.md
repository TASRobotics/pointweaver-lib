# Changelog

## 2027.0.3 (unreleased)

- **Turn direction per point:** `"turnDirection": "cw"` or `"ccw"` in a heading point's params makes an interpolated turn into it go that way round, even when the other way is shorter (`"auto"` or unset keeps the shorter way). Points only; ignored on the path. The heading feedforward follows the same turn. New `TurnDirection` enum and a `PathParams` constructor that takes it; the old constructor still works.

## 2027.0.2

- **Commands v3 adapter** (`pointweaver.command3.PointWeaverCommands`, needs the Commands V3 vendordep): `followPath(name)` requires the drivetrain if it's a `Mechanism` (or pass the mechanism: `followPath(name, swerve)`) and idles the drivetrain however it ends. `registerEvent(name, command)` starts the command on the next scheduler loop as its own command, so it outlives the path (until it finishes or the auto ends) instead of being canceled with it, and refuses a command that would take the drivetrain from a running path or the auto around it. The Commands v2 adapter is unchanged.

## 2027.0.1

- **Pass speeds:** points the robot only passes through no longer get braked toward a stop. Each one gets a speed it may carry past it, limited by the corner (it has to fit inside the early exit range), the next point, and max speed. On a 17-point figure-eight that cut the lap from about 11.7 s to 10.6 s in a sim calibrated to a real robot, with the same shape; max speed now actually matters on straights.
- **Braking on what the robot is doing:** braking uses the measured velocity instead of the last command, starts `responseLag` earlier, and on points it stops at, leaves room to cancel sideways velocity first. A robot coming into the last point sideways no longer swings past it and back.
- **New `responseLag`** in robot.json / `PointWeaverConfig.Builder.responseLag` (default 0.08 s, about what a CTRE swerve measured). `FollowerCore.update` takes the measured velocity; the old overload uses the last command.
- **Interpolated turns finish at the early exit range** instead of at the point, so the heading target no longer jumps when the robot moves on (it jumped about 22° per point with a 0.75 m exit range on 1.5 m spacing).
- **Heading feedforward:** the path's turn rate goes to CTRE's `withTargetRateFeedforward`, so the heading controller no longer lags behind turns (typical error about 0.6° instead of 7.5° on the same figure-eight). Published as `desiredHeadingRateDps`.

## 2027.0.0

- WPILib 2027 alpha-7 and CTRE Phoenix 6 `26.70.0-alpha-2`. Java 25.
- `alpha-8` branch: WPILib 2027 alpha-8, no code changes. Vendordeps still say `"wpilibYear": "2027_alpha7"`, because alpha-8 kept that year.
- `org.wpilib` packages throughout the glue (the move didn't touch `pointweaver.core`). See [docs/migrating-to-2027.md](docs/migrating-to-2027.md).
- `PointWeaverCommands` now needs the Commands V2 vendordep (it's no longer built into WPILib).
- The vendordep is published under `/2027/PointWeaver.json`, so the 2026 one at the site root keeps working.
- `positionTolerance` is a straight-line distance from the point (a circle), not a box on each axis, so a diagonal approach can't stop up to 1.4× further away.
- `interpolate` on the path now turns it on for every heading point, overriding points set to off. Off on the path no longer turns points off; each point decides.
- Without `interpolate`, the robot aims for the next heading point as soon as it leaves the previous one, instead of holding the old heading until the last leg.

## 2026.1.0 (unreleased)

First standalone release, split out of team 8503's robot code.

- Install as a WPILib vendordep. Requires CTRE Phoenix 6.
- Settings live in `deploy/pointweaver/robot.json`, shared with the app's preview. `PointWeaver.configure(drivetrain)` reads it. Paths moved to `deploy/pointweaver/paths/`.
- Speed planning: a braking curve plus final-approach P replaces the plain P on distance (a 4 m line drops from 4.4 s to 2.3 s), with one acceleration limit (`maxAccel`, default 3 m/s²) on the velocity vector for starts, stops and corners. The default translation kP goes 1 → 5.
- New path/point param `maxAccel`. New telemetry: `speed`, `plannedSpeed`.
- `PointWeaver.configure(drivetrain, config)` works with any Phoenix 6 `SwerveDrivetrain`.
- `PathFollower` (`start` / `update` / `stop`) for TimedRobot and OpMode-style code, plus `PointWeaverCommands` for command-based robots.
- Telemetry moved from SmartDashboard keys to NT4 topics under `/PointWeaver`, including Pose2d structs for the robot, target and path.
- No Jackson dependency.
- `interpolate: true` now means a smooth turn with no snapping. It's read from the heading point that ends each stretch.
- The last point always arrives: early exit and min speed are ignored there.
- Translation-only points no longer wait on heading tolerance.
- Events: `registerEvent(name, Runnable)`. Events skipped by early exit still fire.
- `resetPose` path option.
