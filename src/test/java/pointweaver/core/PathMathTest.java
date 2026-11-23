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

    static PathParams turn(TurnDirection dir) {
        return new PathParams(null, null, null, null, null, true, dir);
    }

    @Test
    void turnDeltaGoesTheChosenWayRound() {
        assertEquals(20, PathMath.turnDeltaDeg(170, -170, TurnDirection.AUTO), 1e-9);
        assertEquals(20, PathMath.turnDeltaDeg(170, -170, TurnDirection.CCW), 1e-9);
        assertEquals(-340, PathMath.turnDeltaDeg(170, -170, TurnDirection.CW), 1e-9);
        assertEquals(270, PathMath.turnDeltaDeg(0, -90, TurnDirection.CCW), 1e-9);
        assertEquals(-90, PathMath.turnDeltaDeg(0, -90, TurnDirection.CW), 1e-9);
        assertEquals(180, PathMath.turnDeltaDeg(0, 180, TurnDirection.CCW), 1e-9);
        assertEquals(-180, PathMath.turnDeltaDeg(0, 180, TurnDirection.CW), 1e-9);
        assertEquals(0, PathMath.turnDeltaDeg(45, 405, TurnDirection.CW), 1e-9, "already facing it: no full spin");
        assertEquals(0, PathMath.turnDeltaDeg(45, 45, TurnDirection.CCW), 1e-9);
    }

    @Test
    void forcedTurnDirectionGoesTheLongWayRound() {
        // heading 0 -> heading 90 over 2 m: auto turns +90, CW turns -270
        List<PathPoint> pts = List.of(pt(0, 0, 0.0, null), pt(2, 0, 90.0, turn(TurnDirection.CW)));
        PathDefinition def = new PathDefinition("t", pts, null, false);
        ResolvedParams[] params = PathMath.resolveAll(def);
        double[] prefix = PathMath.prefixDistances(pts);
        assertEquals(-135, PathMath.desiredHeadingDeg(pts, prefix, params, 1, 1, 0, 0, 0, 0), 1e-9, "halfway round clockwise");
        assertEquals(-270, PathMath.desiredHeadingDeg(pts, prefix, params, 1, 2, 0, 0, 0, 0), 1e-9, "= 90, arrived");
        // feedforward turns the same way: -270 deg over 2 m (start to stop, so evenly) at 1 m/s
        assertEquals(-135, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 1, 1, 0, 1, 0, 0, 0, 0), 1e-9);
    }

    /** heading points 1 m apart on a line, each 30 deg further round, early exit 0.3 m, interpolated */
    static List<PathPoint> chain(double... headings) {
        PathParams exit = new PathParams(null, null, null, 0.3, null, true);
        List<PathPoint> pts = new java.util.ArrayList<>();
        for (int i = 0; i < headings.length; i++) pts.add(pt(i, 0, headings[i], exit));
        return pts;
    }

    @Test
    void turnsAreProportionalToDistanceAndChainWithoutPausing() {
        List<PathPoint> pts = chain(0, 30, 60, 90);
        ResolvedParams[] params = PathMath.resolveAll(new PathDefinition("c", pts, null, false));
        double[] prefix = PathMath.prefixDistances(pts);
        // each turn runs point to point (1 m, 30 deg), so the heading is 30 deg per meter all the way along, whichever point
        // the robot is driving to: it moves on 0.3 m early from points 1 and 2 without the heading jumping
        for (double x = 0; x <= 3; x += 0.01) {
            int k = Math.min(3, (int) Math.floor(x + 0.3) + 1);
            assertEquals(30 * x, PathMath.desiredHeadingDeg(pts, prefix, params, k, x, 0, 0, 0, 0), 1e-9, "x=" + x + " k=" + k);
        }
        // the spin rate is the turn's deg/m times the speed: 30 deg/m at 1 m/s, and 60 at 2 m/s
        assertEquals(30, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 2, 1.2, 0, 1, 0, 0, 0, 0), 1e-9);
        assertEquals(60, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 2, 1.2, 0, 2, 0, 0, 0, 0), 1e-9);
        // either side of moving on from point 1 (0.3 m before it): same heading, still turning
        assertEquals(21, PathMath.desiredHeadingDeg(pts, prefix, params, 1, 0.7, 0, 0, 0, 0), 1e-9);
        assertEquals(21, PathMath.desiredHeadingDeg(pts, prefix, params, 2, 0.7, 0, 0, 0, 0), 1e-9);
        assertEquals(30, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 2, 0.7, 0, 1, 0, 0, 0, 0), 1e-9);
    }

    @Test
    void aTurnThatReversesFlipsAtThePoint() {
        List<PathPoint> pts = chain(0, 60, 0, 0);
        ResolvedParams[] params = PathMath.resolveAll(new PathDefinition("r", pts, null, false));
        double[] prefix = PathMath.prefixDistances(pts);
        assertEquals(60, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 1, 0.5, 0, 1, 0, 0, 0, 0), 1e-9);
        assertEquals(60, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 2, 0.8, 0, 1, 0, 0, 0, 0), 1e-9,
            "moved on from point 1 early: still turning into it until level with it");
        assertEquals(-60, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 2, 1.2, 0, 1, 0, 0, 0, 0), 1e-9,
            "past point 1: turning back");
        assertEquals(0, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 3, 2.0, 0, 1, 0, 0, 0, 0), 1e-9, "holding 0 to the end");
    }

    @Test
    void turnDirectionIsPerPointOnly() {
        assertEquals(TurnDirection.AUTO, PathMath.resolve(PathParams.EMPTY, PathParams.EMPTY).turnDirection);
        assertEquals(TurnDirection.CCW, PathMath.resolve(PathParams.EMPTY, turn(TurnDirection.CCW)).turnDirection);
        assertEquals(TurnDirection.CCW, PathMath.resolve(turn(TurnDirection.CW), turn(TurnDirection.CCW)).turnDirection);
        assertEquals(TurnDirection.AUTO, PathMath.resolve(turn(TurnDirection.CW), null).turnDirection, "ignored on the path");
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
    void brakingStartsEarlierByTheResponseLag() {
        // at 2 m/s with 0.08 s of lag, 0.16 m more is gone before the robot reacts
        assertEquals(Math.sqrt(2 * 0.9 * 3 * (2.0 - 2.0 * 0.1)), PathMath.brakingSpeed(2.0, 2.0, 0, 3, 0.02, 0.08), 1e-9);
        assertEquals(PathMath.brakingSpeed(2.0, 2.0, 0, 3, 0.02), PathMath.brakingSpeed(2.0, 2.0, 0, 3, 0.02, 0), 1e-12);
    }

    @Test
    void cornerSpeedFitsTheTurnInsideTheExitCircle() {
        PathPoint a = pt(0, 0, null, null);
        PathPoint corner = pt(2, 0, null, null);
        assertEquals(Double.POSITIVE_INFINITY, PathMath.cornerSpeed(a, corner, pt(4, 0, null, null), 0.5, 3, 0), "straight on");
        assertEquals(Math.sqrt(0.9 * 3 * 0.5 / 1.0), PathMath.cornerSpeed(a, corner, pt(2, 2, null, null), 0.5, 3, 0), 1e-9, "90 deg");
        assertEquals(Math.sqrt(0.9 * 3 * 0.5 / Math.tan(Math.toRadians(22.5))),
            PathMath.cornerSpeed(a, corner, pt(4, 2 * Math.tan(Math.toRadians(45)), null, null), 0.5, 3, 0), 1e-9, "45 deg");
        assertTrue(PathMath.cornerSpeed(a, corner, pt(0, 0.001, null, null), 0.5, 3, 0) < 0.05, "doubling back");
        assertEquals(0, PathMath.cornerSpeed(a, a, corner, 0.5, 3, 0), "zero-length side");
        // with lag the turn starts late: v^2 = k (r - v lag), here k = 2.7
        double v = PathMath.cornerSpeed(a, corner, pt(2, 2, null, null), 0.5, 3, 0.1);
        assertEquals(2.7 * (0.5 - v * 0.1), v * v, 1e-9);
        assertTrue(v < Math.sqrt(0.9 * 3 * 0.5), "slower than with no lag");
    }

    @Test
    void passSpeedsStopAtTheEndsAndAtHeadingPointsWithoutExit() {
        PathParams exit = new PathParams(null, null, null, 0.5, null, null);
        List<PathPoint> pts = List.of(
            pt(0, 0, 0.0, null),
            pt(2, 0, null, null),          // straight through, no exit range: only the room to the next point limits it
            pt(4, 0, 0.0, null),           // heading point without an exit range: stop and turn
            pt(6, 0, null, exit),          // 90 deg corner inside a 0.5 m exit range
            pt(6, 2, 90.0, null));
        PathDefinition def = new PathDefinition("p", pts, null, false);
        double[] pass = PathMath.passSpeeds(pts, PathMath.resolveAll(def), 5.0, 3.0, 0.05, 0);
        assertEquals(0, pass[0], "first point");
        assertEquals(0, pass[4], "last point");
        assertEquals(0, pass[2], "heading point without an exit range");
        assertEquals(Math.sqrt(2 * 0.9 * 3 * (2.0 - 0.05)), pass[1], 1e-9, "slow enough to stop at the heading point");
        assertEquals(Math.sqrt(0.9 * 3 * 0.5), pass[3], 1e-9, "the corner");

        PathDefinition capped = new PathDefinition("c", pts, new PathParams(null, 3.0, null, null, null, null), false);
        assertEquals(3.0, PathMath.passSpeeds(pts, PathMath.resolveAll(capped), 5.0, 3.0, 0.05, 0)[1], 1e-9, "the path's max speed");
    }

    @Test
    void interpolatedTurnFinishesAtThePointEvenWithEarlyExit() {
        // heading 0 -> heading 90 over 2 m; the robot moves on from point 1 0.5 m early, at 1.5 m
        PathParams exit = new PathParams(null, null, null, 0.5, null, true);
        List<PathPoint> pts = List.of(pt(0, 0, 0.0, null), pt(2, 0, 90.0, exit), pt(4, 0, 90.0, exit));
        PathDefinition def = new PathDefinition("t", pts, null, false);
        ResolvedParams[] params = PathMath.resolveAll(def);
        double[] prefix = PathMath.prefixDistances(pts);
        assertEquals(45, PathMath.desiredHeadingDeg(pts, prefix, params, 1, 1.0, 0, 0, 0, 0), 1e-9, "halfway");
        assertEquals(67.5, PathMath.desiredHeadingDeg(pts, prefix, params, 1, 1.5, 0, 0, 0, 0), 1e-9, "3/4 of the way when it moves on");
        // the handoff to point 2 doesn't make the heading jump: it keeps turning into point 1 until it's level with it
        assertEquals(67.5, PathMath.desiredHeadingDeg(pts, prefix, params, 2, 1.5, 0, 0, 0, 0), 1e-9);
        assertEquals(90, PathMath.desiredHeadingDeg(pts, prefix, params, 2, 2.0, 0, 0, 0, 0), 1e-9, "facing 90 at point 1");
        assertEquals(90, PathMath.desiredHeadingDeg(pts, prefix, params, 2, 3.0, 0, 0, 0, 0), 1e-9, "and holding it after");
    }

    @Test
    void headingRateIsHowFastTheTurnMovesAlongThePath() {
        List<PathPoint> pts = List.of(pt(0, 0, 0.0, null), pt(2, 0, 90.0, interp(true)));
        PathDefinition def = new PathDefinition("t", pts, null, false);
        ResolvedParams[] params = PathMath.resolveAll(def);
        double[] prefix = PathMath.prefixDistances(pts);
        // the turn is proportional to distance: 90 deg over 2 m is 45 deg/m, so 45 deg/s at 1 m/s
        assertEquals(45, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 1, 1, 0, 1, 0, 0, 0, 0), 1e-9);
        assertEquals(0, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 1, 1, 0, 0, 0, 0, 0, 0), 1e-9, "standing still");
        assertEquals(0, PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, 1, 1, 0, 0, 1, 0, 0, 0), 1e-9, "moving sideways");

        // same between points the robot drives through: turning 90 deg over 0.7..2.7 m is 45 deg/m, so 45 deg/s at 1 m/s
        PathParams pass = new PathParams(null, null, null, 0.3, null, true);
        List<PathPoint> mid = List.of(pt(-1, 0, 0.0, null), pt(0, 0, 0.0, pass), pt(2, 0, 90.0, pass), pt(3, 0, 90.0, pass));
        ResolvedParams[] midParams = PathMath.resolveAll(new PathDefinition("m", mid, null, false));
        assertEquals(45, PathMath.desiredHeadingRateDegPerSec(mid, PathMath.prefixDistances(mid), midParams, 2, 0.7, 0, 1, 0, -1, 0, 0), 1e-9);

        List<PathPoint> snap = List.of(pt(0, 0, 0.0, null), pt(2, 0, 90.0, interp(false)));
        PathDefinition snapDef = new PathDefinition("s", snap, null, false);
        assertEquals(0, PathMath.desiredHeadingRateDegPerSec(snap, PathMath.prefixDistances(snap), PathMath.resolveAll(snapDef),
            1, 1, 0, 1, 0, 0, 0, 0), 1e-9, "no interpolation: aims straight for the heading, no feedforward");
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
