package pointweaver.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

/**
 * Drives {@link FollowerCore} with the same kinematic model as the PointWeaver app's ghost
 * (src/sim/simulate.ts): the robot moves at the commanded velocity, and heading follows a P controller
 * like CTRE's. If the timing numbers here change, the robot and the app's ghost no longer agree.
 */
class FollowerCoreSimTest {
    static final double DT = 0.02;
    static final double ROT_KP = 2.0;
    static final double MAX_ANGULAR_RATE = Math.toRadians(918);

    static FollowerSettings settings(double accel) {
        return new FollowerSettings(6.48, accel, 5.0, 0, 0, 0.05, 2.0, DT);
    }

    static final class Run {
        double duration;
        final List<Double> pointTimes = new ArrayList<>();
        final List<String> events = new ArrayList<>();
        double endX, endY, endHeading;
        /** biggest change in commanded velocity in one loop, excluding the final stop (m/s) */
        double maxDeltaV;
        /** furthest past the last point, measured along the last segment (m) */
        double overshoot;
    }

    static Run simulate(PathDefinition def, FollowerSettings settings) {
        Run run = new Run();
        FollowerCore core = new FollowerCore(def, settings, run.events::add);
        PathPoint first = def.points.get(0);
        double x = first.x, y = first.y, h = first.theta != null ? first.theta : 0;
        double t = 0, lastVx = 0, lastVy = 0;
        core.start(x, y, h, t);
        int index = 0;
        PathPoint last = def.points.get(def.points.size() - 1);
        PathPoint beforeLast = def.points.get(Math.max(0, def.points.size() - 2));
        double lastLen = PathMath.dist(beforeLast.x, beforeLast.y, last.x, last.y);
        for (int step = 0; step < 100_000; step++) {
            FollowerCore.Output out = core.update(x, y, h, t);
            if (!out.finished) run.maxDeltaV = Math.max(run.maxDeltaV, Math.hypot(out.vx - lastVx, out.vy - lastVy));
            lastVx = out.vx;
            lastVy = out.vy;
            double err = Math.toRadians(PathMath.shortestAngleDeg(h, out.headingDeg));
            double omega = PathMath.clamp(ROT_KP * err, -MAX_ANGULAR_RATE, MAX_ANGULAR_RATE);
            x += out.vx * DT;
            y += out.vy * DT;
            h = ((h + Math.toDegrees(omega * DT) + 180) % 360 + 360) % 360 - 180;
            t += DT;
            if (lastLen > PathMath.EPS && core.targetIndex() == def.points.size() - 1) {
                double along = ((x - beforeLast.x) * (last.x - beforeLast.x) + (y - beforeLast.y) * (last.y - beforeLast.y)) / lastLen;
                run.overshoot = Math.max(run.overshoot, along - lastLen);
            }
            if (out.finished || core.targetIndex() != index) {
                run.pointTimes.add(t);
                index = core.targetIndex();
            }
            if (out.finished) break;
        }
        run.duration = t;
        run.endX = x;
        run.endY = y;
        run.endHeading = h;
        return run;
    }

    static String fmt(List<Double> v) {
        return v.stream().map(d -> String.format("%.2f", d)).collect(Collectors.joining(" "));
    }

    static PathDefinition testpath() throws Exception {
        return PathParser.parse(PathParserTest.fixture("testpath"));
    }

    static PathDefinition line() {
        return new PathDefinition("line", List.of(new PathPoint(1, 1, 0.0, null, null), new PathPoint(5, 1, 0.0, null, null)), null, false);
    }

    static PathDefinition corner() {
        PathParams p = new PathParams(null, 4.0, null, 0.5, null, null);
        return new PathDefinition("corner", List.of(new PathPoint(1, 1, 0.0, null, null), new PathPoint(5, 1, 0.0, null, null),
            new PathPoint(5, 5, 90.0, null, null)), p, false);
    }

    @Test
    void matchesTheAppGhost() throws Exception {
        // the exact numbers the app's simulatePath() prints for these paths with these settings
        Run run = simulate(testpath(), settings(3.0));
        assertEquals("6.66", String.format("%.2f", run.duration));
        assertEquals("0.02 0.74 1.44 2.16 2.84 3.54 4.22 6.66", fmt(run.pointTimes));
        assertEquals(4.707, run.endX, 0.05);
        assertEquals(1.293, run.endY, 0.05);
        assertEquals(135, run.endHeading, 2.0);

        assertEquals("0.02 2.26", fmt(simulate(line(), settings(3.0)).pointTimes));
        assertEquals("0.02 1.82 4.14", fmt(simulate(corner(), settings(3.0)).pointTimes));
    }

    @Test
    void neverExceedsTheAccelerationLimit() throws Exception {
        for (double a : new double[] {3, 4}) {
            for (PathDefinition def : List.of(testpath(), line(), corner())) {
                Run run = simulate(def, settings(a));
                assertTrue(run.maxDeltaV <= a * DT + 1e-9, def.name + " a=" + a + " jumped " + run.maxDeltaV / DT + " m/s^2");
            }
        }
    }

    @Test
    void stopsWithoutOvershooting() throws Exception {
        for (double a : new double[] {3, 4}) {
            for (PathDefinition def : List.of(testpath(), line(), corner())) {
                Run run = simulate(def, settings(a));
                assertTrue(run.overshoot < 0.01, def.name + " a=" + a + " overshot " + run.overshoot + " m");
            }
        }
    }

    @Test
    void straightLineDoesNotCrawl() {
        // 4 m from rest: about 2.2 s at 3 m/s^2 (it used to take 4.38 s)
        assertTrue(simulate(line(), settings(3.0)).duration < 2.5);
    }

    @Test
    void pathAccelerationOverrideIsRespectedAndCapped() {
        PathDefinition gentle = new PathDefinition("g", line().points, new PathParams(null, null, 1.0, null, null, null), false);
        assertTrue(simulate(gentle, settings(3.0)).maxDeltaV <= 1.0 * DT + 1e-9);
        PathDefinition greedy = new PathDefinition("g", line().points, new PathParams(null, null, 10.0, null, null, null), false);
        assertTrue(simulate(greedy, settings(3.0)).maxDeltaV <= 3.0 * DT + 1e-9, "can't go above the robot's limit");
    }

    @Test
    void startingWhileMovingSlewsFromTheMeasuredVelocity() {
        FollowerCore core = new FollowerCore(line(), settings(3.0), null);
        core.start(1, 1, 0, -2.0, 0, 0); // moving away from the path at 2 m/s
        FollowerCore.Output out = core.update(1, 1, 0, 0);
        assertEquals(-2.0 + 3.0 * DT, out.vx, 1e-9, "only one loop's worth of acceleration toward the target");
    }

    @Test
    void firesEventsInOrderEvenWhenEarlyExitSkipsThem() {
        PathParams exit = new PathParams(null, null, null, 0.6, null, null);
        PathDefinition def = new PathDefinition("e", List.of(
            new PathPoint(0, 0, 0.0, null, List.of(new PathEvent("start", 0.0), new PathEvent("half", 0.5))),
            new PathPoint(2, 0, null, exit, List.of(new PathEvent("late", 0.99))),
            new PathPoint(4, 0, 0.0, null, List.of(new PathEvent("done", 0.0)))), exit, false);
        Run run = simulate(def, settings(3.0));
        assertEquals(List.of("start", "half", "late", "done"), run.events);
    }

    @Test
    void resetPoseUsesFirstPoint() {
        PathDefinition reset = new PathDefinition("r", List.of(new PathPoint(1, 2, null, null, null)), null, true);
        FollowerCore.Pose p = new FollowerCore(reset, settings(3.0), null).resetPose(33);
        assertNotNull(p);
        assertEquals(1, p.x);
        assertEquals(2, p.y);
        assertEquals(33, p.headingDeg, "no heading on point 0 keeps the gyro heading");

        PathDefinition noReset = new PathDefinition("n", reset.points, null, false);
        assertNull(new FollowerCore(noReset, settings(3.0), null).resetPose(0));
    }

    @Test
    void emptyPathFinishesImmediately() {
        FollowerCore core = new FollowerCore(PathDefinition.empty("x"), settings(3.0), null);
        core.start(0, 0, 0, 0);
        assertTrue(core.isFinished());
        FollowerCore.Output out = core.update(0, 0, 0, 0);
        assertTrue(out.finished);
        assertEquals(0, out.vx);
    }
}
