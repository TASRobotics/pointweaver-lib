# Moving to WPILib 2027 (alpha-7)

The library is laid out so the 2027 move is mostly a dependency and import swap.

| 2027 change | Where it hits PointWeaver |
|---|---|
| Java packages `edu.wpi.first.*` → `org.wpilib.*`, constants ALL_CAPS | Only the glue: `PointWeaver`, `PointWeaverConfig`, `PathFollower`, `WeaverTelemetry`, `command/*`. `pointweaver.core` has no WPILib imports. |
| Jackson removed (Avaje Jsonb) | Nothing. The core has its own JSON reader. |
| SmartDashboard / Sendable replaced by Telemetry / Tunables | Nothing. Telemetry is raw NT4 publishers in `WeaverTelemetry`. |
| `OpModeRobot`: OpModes are `start()` / `periodic()` / `end()` | `PathFollower` already has `start()` / `update()` / `stop()`. |
| Java 25 | The code is Java 17 compatible. Bump `options.release`. |
| Vendordeps from alpha-6 and earlier don't load | Needs a Phoenix 6 build for alpha-7 from CTRE. **This is the external blocker.** |

## Checklist

1. **`build.gradle`:**
   - `wpilibVersion = '2027.0.0-alpha-7'`, using the new artifact coordinates from the alpha-7 release.
   - `phoenixVersion` = CTRE's 2027 alpha build.
   - `frcYear = '2027'`.
   - `options.release = 25`.
2. **Update imports in the glue files.** The VS Code importer's package mapping is a good reference.
3. **`command/`:** keep the Commands v2 adapter if v2 is still shipped, and/or add a Commands v3 one. OpMode users don't need either.
4. **Vendordep:** bump the version to `2027.x` and update the Phoenix 6 `requires` entry (uuid, file name, URL) to CTRE's 2027 vendordep.
5. **Release** from a `2027` branch, so 2026 teams keep getting `2026.x` from `main`.

## A 2027 auto, for reference

```java
@Autonomous(name = "Test Path")
public class TestPathAuto implements OpMode {
    private final PathFollower follower = PointWeaver.follower("testpath");

    @Override public void start() { follower.start(); }
    @Override public void periodic() { follower.update(); }
    @Override public void end() { follower.stop(); }
}
```

`PointWeaver.configure(...)` goes in the `OpModeRobot` subclass constructor.
