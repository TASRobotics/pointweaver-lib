# Changelog

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
