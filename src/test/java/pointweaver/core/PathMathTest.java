package pointweaver.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import pointweaver.core.PathMath.ResolvedParams;

class PathMathTest {
    static PathPoint pt(double x, double y, Double theta, PathParams params) {
        return new PathPoint(x, y, theta, params, null);
    }

    static PathParams interp(boolean on) {
        return new PathParams(null, null, null, null, null, on);
    }

    @Test
    void shortestAngleWraps() {
        assertEquals(20, PathMath.shortestAngleDeg(170, -170), 1e-9);
        assertEquals(-20, PathMath.shortestAngleDeg(-170, 170), 1e-9);
        assertEquals(-180, PathMath.shortestAngleDeg(0, 180), 1e-9);
        assertEquals(10, PathMath.shortestAngleDeg(-370, -360), 1e-9);
    }

    @Test
    void pointParamsOverridePathParams() {
        PathParams path = new PathParams(1.0, 3.0, 4.0, 0.5, 8.0, null);
        ResolvedParams r = PathMath.resolve(path, new PathParams(null, 2.0, 2.5, null, null, null));
        assertEquals(1.0, r.minSpeed);
        assertEquals(2.0, r.maxSpeed);
        assertEquals(2.5, r.maxAccel);
        assertEquals(0.5, r.earlyExitRange);
        assertEquals(8.0, r.timeout);

        ResolvedParams defaults = PathMath.resolve(PathParams.EMPTY, PathParams.EMPTY);
        assertEquals(0.0, defaults.minSpeed);
        assertEquals(Double.POSITIVE_INFINITY, defaults.maxSpeed);
        assertEquals(Double.POSITIVE_INFINITY, defaults.maxAccel);
        assertFalse(defaults.hasEarlyExit());
        assertFalse(defaults.interpolate);
    }

    @Test
    void pathInterpolateTurnsEveryPointOn() {
        // on for the path: every point interpolates, even one set to off
        assertTrue(PathMath.resolve(interp(true), interp(false)).interpolate);
        assertTrue(PathMath.resolve(interp(true), null).interpolate);
        // off or unset for the path: each point decides, default off
        assertTrue(PathMath.resolve(interp(false), interp(true)).interpolate);
        assertTrue(PathMath.resolve(null, interp(true)).interpolate);
        assertFalse(PathMath.resolve(interp(false), interp(false)).interpolate);
        assertFalse(PathMath.resolve(interp(false), null).interpolate);
    }

    @Test
    void lastPointIgnoresMinSpeedAndRobotMaxCaps() {
        ResolvedParams p = PathMath.resolve(new PathParams(2.0, 10.0, null, null, null, null), null);
        assertEquals(2.0, PathMath.clampSpeed(0.1, p, false, 5.0), 1e-9);
        assertEquals(0.1, PathMath.clampSpeed(0.1, p, true, 5.0), 1e-9);
        assertEquals(5.0, PathMath.clampSpeed(50, p, false, 5.0), 1e-9);
        assertEquals(0.0, PathMath.clampSpeed(0.0, p, false, 5.0), 1e-9);
    }

    @Test
    void finishRules() {
        ResolvedParams exit = PathMath.resolve(new PathParams(null, null, null, 1.0, 2.0, null), null);
        PathPoint heading = pt(0, 0, 90.0, null);
        PathPoint trans = pt(0, 0, null, null);

        assertTrue(PathMath.isPointFinished(heading, exit, false, 0.5, 0, 0, 0, 0.05, 2), "early exit");
        assertFalse(PathMath.isPointFinished(heading, exit, true, 0.5, 0, 0, 0, 0.05, 2), "last point ignores early exit");
        assertTrue(PathMath.isPointFinished(heading, exit, true, 5, 5, 0, 2.5, 0.05, 2), "timeout");

        ResolvedParams none = PathMath.resolve(null, null);
        assertFalse(PathMath.isPointFinished(heading, none, false, 0.01, 0, 45, 0, 0.05, 2), "heading point needs heading");
        assertTrue(PathMath.isPointFinished(heading, none, false, 0.01, 0, 89, 0, 0.05, 2));
        assertTrue(PathMath.isPointFinished(trans, none, false, 0.01, 0, 45, 0, 0.05, 2), "translation point skips heading");

        // position tolerance is a circle: 4 cm off on both axes is 5.7 cm away, outside a 5 cm tolerance
        assertFalse(PathMath.isPointFinished(trans, none, false, 0.04, 0.04, 0, 0, 0.05, 2), "corner of the old box");
        assertTrue(PathMath.isPointFinished(trans, none, false, 0.03, 0.03, 0, 0, 0.05, 2), "4.2 cm away");
    }

    @Test
    void interpolationNeverSnaps() {
        // heading 0 -> translation -> heading 90, interpolate on the end point
        List<PathPoint> pts = List.of(pt(0, 0, 0.0, null), pt(1, 0, null, null), pt(2, 0, 90.0, interp(true)));
        PathDefinition def = new PathDefinition("t", pts, null, false);
        ResolvedParams[] params = PathMath.resolveAll(def);
        double[] prefix = PathMath.prefixDistances(pts);

        // at point 1, heading is the same whether we're still driving to 1 or already driving to 2
        double toward1 = PathMath.desiredHeadingDeg(pts, prefix, params, 1, 1, 0, 0, 0, 0);
        double toward2 = PathMath.desiredHeadingDeg(pts, prefix, params, 2, 1, 0, 0, 0, 0);
        assertEquals(45, toward1, 1e-9);
        assertEquals(toward1, toward2, 1e-9);
        assertEquals(90, PathMath.desiredHeadingDeg(pts, prefix, params, 2, 2, 0, 0, 0, 0), 1e-9);
    }

    @Test
    void withoutInterpolationAimsForTheNextHeadingRightAway() {
        List<PathPoint> pts = List.of(pt(0, 0, 0.0, null), pt(1, 0, null, null), pt(2, 0, 90.0, interp(false)));
        PathDefinition def = new PathDefinition("t", pts, null, false);
        ResolvedParams[] params = PathMath.resolveAll(def);
        double[] prefix = PathMath.prefixDistances(pts);
        // still driving to point 0: its own heading (not the robot's starting 45)
        assertEquals(0, PathMath.desiredHeadingDeg(pts, prefix, params, 0, 0, 0, 0, 0, 45), 1e-9);
        // left point 0, driving to the translation point: already aiming for point 2's heading
        assertEquals(90, PathMath.desiredHeadingDeg(pts, prefix, params, 1, 0.5, 0, 0, 0, 0), 1e-9);
        assertEquals(90, PathMath.desiredHeadingDeg(pts, prefix, params, 2, 1.0, 0, 0, 0, 0), 1e-9);
    }

    @Test
    void brakingSpeedCanStillStopAtThePoint() {
        // v^2 = 2 * 0.9a * d, with one loop of travel taken off d
        assertEquals(Math.sqrt(2 * 0.9 * 3 * 2.0), PathMath.brakingSpeed(2.0, 0, 0, 3, 0.02), 1e-9);
        assertEquals(Math.sqrt(2 * 0.9 * 3 * (2.0 - 0.04)), PathMath.brakingSpeed(2.0, 2.0, 0, 3, 0.02), 1e-9);
        assertEquals(0, PathMath.brakingSpeed(0.01, 1.0, 0, 3, 0.02), 1e-9);
        assertEquals(1.5, PathMath.brakingSpeed(0, 0, 1.5, 3, 0.02), 1e-9, "end speed when already there");
    }

    @Test
    void slewLimitsTheWholeVector() {
        double[] v = PathMath.slew(0, 0, 3, 4, 0.5);
        assertEquals(0.5, Math.hypot(v[0], v[1]), 1e-9);
        assertEquals(0.3, v[0], 1e-9);
        double[] turn = PathMath.slew(2, 0, 0, 2, 0.06); // 90 degree turn: only 0.06 m/s of change per loop
        assertEquals(0.06, Math.hypot(turn[0] - 2, turn[1]), 1e-9);
        double[] close = PathMath.slew(1, 1, 1.01, 1, 0.5);
        assertEquals(1.01, close[0], 1e-12);
    }
}
