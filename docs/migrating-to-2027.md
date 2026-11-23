# WPILib 2027 (alpha-7)

This `alpha-8` branch builds against WPILib `2027.0.0-alpha-8` and CTRE Phoenix 6 `26.70.0-alpha-2`; `main` is the same code on alpha-7.
The legacy 2026 version is on the `2026` branch. `pointweaver.core` didn't change at all; only the glue did.

## What changed from 2026

| 2026 | 2027 alpha-7 |
|---|---|
| Java 17 | Java 25 (`options.release = 25`, CI on JDK 25) |
| `edu.wpi.first.*` artifacts | `org.wpilib.*` artifacts, plus `telemetry-java` and `tunables-java` (Commands V2's `Command` implements their interfaces) |
| `edu.wpi.first.wpilibj.DriverStation.reportError/reportWarning` | `org.wpilib.driverstation.DriverStationErrors` |
| `edu.wpi.first.wpilibj.Filesystem`, `Timer` | `org.wpilib.system.Filesystem`, `Timer` |
| `Timer.getFPGATimestamp()` | `Timer.getTimestamp()` |
| `edu.wpi.first.math.geometry.*`, `edu.wpi.first.units.*` | `org.wpilib.math.geometry.*`, `org.wpilib.units.*` |
| `ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, angle)` | `state.Velocity.toFieldRelative(angle)` (`ChassisVelocities`, fields `vx` / `vy`) |
| `edu.wpi.first.networktables.*` | `org.wpilib.networktables.*` |
| `edu.wpi.first.wpilibj2.command.*` (built in) | `org.wpilib.command2.*` (the Commands V2 vendordep) |
| Vendordep `"frcYear": "2026"` at the site root | `"wpilibYear": "2027_alpha7"`, published under `/2027/` |
| Phoenix 6 `26.1.1` | Phoenix 6 `26.70.0-alpha-2` (the swerve API PointWeaver uses is unchanged) |

## For robot projects

Your own code needs the same kind of changes. The example robot shows a full move (compare its `main` with its `2026` branch):

- GradleRIO plugin `org.wpilib.GradleRIO` `2027.0.0-alpha-7`, Gradle 9.4.1, Java 25, SystemCore deploy target.
- Tuner X files: `org.wpilib` imports, `.withCANBusName(kCANBus.getName())` → `.withNetwork(kCANBus)`,
  and `new CANBus(name, hootFile)` → `new CANBus(name)`.
- Autos as OpModes: `Robot extends OpModeRobot`, and each auto is an `@Autonomous` class implementing `OpMode`:

```java
@Autonomous(name = "Test Path")
public class TestPathAuto implements OpMode {
    private final PathFollower follower = PointWeaver.follower("testpath");

    @Override public void start() { follower.start(); }
    @Override public void periodic() { follower.update(); }
    @Override public void end() { follower.stop(); }
}
```

`PointWeaver.configure(...)` goes in the `OpModeRobot` subclass constructor. `OpModeRobot` finds `@Autonomous`
classes in its own package and subpackages, and creates one when it's selected on the Driver Station.

## Not done yet

- CTRE's `Phoenix6-frc2027-latest.json` (the vendordep's `requires` link for Phoenix) doesn't resolve yet.
  It's only used to prompt installing Phoenix; the UUID check works without it.

## Alpha-7 → alpha-8 (the `alpha-8` branch)

- Only versions change: `wpilibVersion = '2027.0.0-alpha-8'` here, GradleRIO `2027.0.0-alpha-8` in robot projects. No code changes.
- Vendordeps keep `"wpilibYear": "2027_alpha7"`. Alpha-8 kept that year, and GradleRIO alpha-8 rejects vendordeps marked
  `2027_alpha8`. That's also why alpha-7 vendordeps such as Phoenix 6 `26.70.0-alpha-2` still load (and they run fine).
- The robot needs SystemCore image 14 or later and FIRST Driver Station alpha 7 or later.
