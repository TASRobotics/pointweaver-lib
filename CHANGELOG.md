# Changelog

## 2027.0.0 (unreleased)

- WPILib 2027 alpha-7 and CTRE Phoenix 6 `26.70.0-alpha-2`. Java 25.
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
