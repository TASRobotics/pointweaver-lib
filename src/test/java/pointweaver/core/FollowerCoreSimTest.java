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
 * (src/sim/simulate.ts): instant translation, heading follows a P controller like CTRE's.
 * If these numbers change, the robot and the app's ghost no longer agree.
 */
class FollowerCoreSimTest {
    // ROBOT in the app's src/constants.ts
    static final FollowerSettings SETTINGS = new FollowerSettings(6.48, 1.0, 0, 0, 0.05, 2.0, 0.02);
    static final double ROT_KP = 2.0;
    static final double MAX_ANGULAR_RATE = 16;
    static final double DT = 0.02;

    static final class Run {
        double duration;
        final List<Double> pointTimes = new ArrayList<>();
        final List<String> events = new ArrayList<>();
        double endX, endY, endHeading;
    }

    static Run simulate(PathDefinition def) {
        Run run = new Run();
        FollowerCore core = new FollowerCore(def, SETTINGS, run.events::add);
        PathPoint first = def.points.get(0);
        double x = first.x, y = first.y, h = first.theta != null ? first.theta : 0;
        double t = 0;
        core.start(x, y, h, t);
        int index = 0;
        for (int step = 0; step < 100_000; step++) {
            FollowerCore.Output out = core.update(x, y, h, t);
            double err = Math.toRadians(PathMath.shortestAngleDeg(h, out.headingDeg));
            double omega = PathMath.clamp(ROT_KP * err, -MAX_ANGULAR_RATE, MAX_ANGULAR_RATE);
            x += out.vx * DT;
            y += out.vy * DT;
            h = ((h + Math.toDegrees(omega * DT) + 180) % 360 + 360) % 360 - 180;
            t += DT;
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

    @Test
    void matchesTheAppGhostOnTestpath() throws Exception {
        Run run = simulate(PathParser.parse(PathParserTest.fixture("testpath")));
        // same numbers the app's simulatePath() prints for this file
        assertEquals("7.52", String.format("%.2f", run.duration));
        assertEquals("0.02 0.42 1.06 1.80 2.36 3.10 3.64 7.52", fmt(run.pointTimes));
        assertEquals(4.707, run.endX, 0.05);
        assertEquals(1.293, run.endY, 0.05);
        assertEquals(135, run.endHeading, 2.0);
    }

    @Test
    void firesEventsInOrderEvenWhenEarlyExitSkipsThem() {
        PathParams exit = new PathParams(null, null, 0.6, null, null);
        PathDefinition def = new PathDefinition("e", List.of(
            new PathPoint(0, 0, 0.0, null, List.of(new PathEvent("start", 0.0), new PathEvent("half", 0.5))),
            new PathPoint(2, 0, null, exit, List.of(new PathEvent("late", 0.99))),
            new PathPoint(4, 0, 0.0, null, List.of(new PathEvent("done", 0.0)))), exit, false);
        Run run = simulate(def);
        assertEquals(List.of("start", "half", "late", "done"), run.events);
    }

    @Test
    void resetPoseUsesFirstPoint() {
        PathDefinition reset = new PathDefinition("r", List.of(new PathPoint(1, 2, null, null, null)), null, true);
        FollowerCore.Pose p = new FollowerCore(reset, SETTINGS, null).resetPose(33);
        assertNotNull(p);
        assertEquals(1, p.x);
        assertEquals(2, p.y);
        assertEquals(33, p.headingDeg, "no heading on point 0 keeps the gyro heading");

        PathDefinition noReset = new PathDefinition("n", reset.points, null, false);
        assertNull(new FollowerCore(noReset, SETTINGS, null).resetPose(0));
    }

    @Test
    void emptyPathFinishesImmediately() {
        FollowerCore core = new FollowerCore(PathDefinition.empty("x"), SETTINGS, null);
        core.start(0, 0, 0, 0);
        assertTrue(core.isFinished());
        FollowerCore.Output out = core.update(0, 0, 0, 0);
        assertTrue(out.finished);
        assertEquals(0, out.vx);
    }
}
