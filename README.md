# PointWeaver

Point-to-point path following for **CTRE Phoenix 6 swerve** drivetrains (FRC, WPILib 2026).
Draw paths in the PointWeaver app, drop them in your deploy folder, follow them with a few lines of code.

- Drives straight at each point in order. Speed comes from a P(ID) on distance, clamped per point.
- Heading uses CTRE's `FieldCentricFacingAngle`, optionally interpolated smoothly between heading points.
- Per-point overrides for speed, early exit and timeout. Named events and optional odometry reset.
- Works with plain `TimedRobot`, command-based, and (later) 2027 OpModes.

## Install

1. Install CTRE Phoenix 6 if you haven't.
2. In VS Code: **WPILib: Manage Vendor Libraries → Install new library (online)** and paste:
   ```
   https://tasrobotics.github.io/pointweaver-lib/PointWeaver.json
   ```

## Use it

Paths live in `src/main/deploy/pointweaver/<name>.json`, which is where the app saves them.

```java
public class Robot extends TimedRobot {
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private PathFollower auto;

    public Robot() {
        PointWeaver.configure(drivetrain, PointWeaverConfig.builder(TunerConstants.kSpeedAt12Volts).build());
        PointWeaver.registerEvent("intake", () -> intake.start());
    }

    @Override public void autonomousInit() {
        auto = PointWeaver.follower("testpath");
        auto.start();
    }

    @Override public void autonomousPeriodic() {
        auto.update(); // until auto.isFinished()
    }

    @Override public void autonomousExit() {
        auto.stop();
    }
}
```

**Command-based:**

```java
PointWeaverCommands.registerEvent("intake", intake.runCommand()); // must not require the drivetrain
Command auto = PointWeaverCommands.followPath("testpath");
```

If a path file is missing or broken, the error goes to the Driver Station and the robot stays still. It doesn't crash.

## Config

| Builder method | Default | What it does |
|---|---|---|
| `builder(maxSpeed)` | required | Drivetrain top speed. Paths can only go slower. |
| `translationPID(p, i, d)` | 1, 0, 0 | Speed (m/s) per meter of distance to the point. |
| `headingPID(p, i, d)` | 2, 0, 0 | CTRE heading controller gains. |
| `positionTolerance(m)` | 0.05 | A point is reached inside this box (per axis). |
| `headingTolerance(deg)` | 2 | Heading points also need this. |
| `pathFolder(name)` | `pointweaver` | Folder inside `deploy/`. |
| `telemetry(on)` / `telemetryTable(name)` | on / `PointWeaver` | NetworkTables output. |

## Path rules

The JSON format and every field are described in [docs/path-format.md](docs/path-format.md). The short version:

- Points are driven to in order. A point **without** a heading is translation-only.
- `interpolate` on a heading point means "turn smoothly into this heading" across the stretch since the previous heading point. Off means hold, then snap on the last leg.
- `earlyExitRange` moves on early, which is good for flowing through points. The **last point always arrives**: early exit and min speed don't apply to it.
- Translation-only points never wait on heading.
- Events fire at a fraction of the way to the next point. Anything skipped by early exit still fires when the point finishes.
- `resetPose: true` resets odometry to point 0 when the path starts.

## Telemetry

Under `/PointWeaver/` on NetworkTables:
- `pose`, `target`, `path`: Pose2d structs, so AdvantageScope and Elastic can draw them on a field.
- `activePath`, `running`, `targetIndex`, `distance`, `desiredHeadingDeg`, `vx`, `vy`, `lastEvent`.

## Developing

```
./gradlew build      # compile + tests (no WPILib install needed)
```

- `pointweaver.core` is plain Java (no WPILib, no CTRE). All the following logic lives there and is unit tested.
- `FollowerCoreSimTest` pins the follower to the PointWeaver app's ghost, so the app preview and the robot can't drift apart.
- The rest of `pointweaver` is thin glue: CTRE drivetrain, NetworkTables, deploy folder. `pointweaver.command` is the optional Commands v2 adapter.
- **Testing in a robot project without publishing:** put this repo next to it and add `includeBuild('../pointweaver-lib')` to its `settings.gradle`. Gradle uses the local source instead of the released jar.
- **Releasing:** push a tag like `v2026.1.0`. GitHub Actions publishes the jar and the vendordep to GitHub Pages. Enable Pages on the `gh-pages` branch once.
- **WPILib 2027:** see [docs/migrating-to-2027.md](docs/migrating-to-2027.md).

MIT licensed.
