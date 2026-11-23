package pointweaver.core;

import java.util.List;

/**
 * Path math used by {@link FollowerCore}. No WPILib/CTRE stuff in here on purpose.
 *
 * <p>!!! The PointWeaver app's ghost (src/sim/follower.ts) is a 1:1 port of this file.
 * If you change the logic here, change it there too, or the ghost lies to you.
 */
final class PathMath {
    static final double EPS = 1e-9;
    /** Braking is planned with this share of the acceleration limit; the rest is left for steering. */
    static final double BRAKE_MARGIN = 0.9;

    private PathMath() {}

    /** Signed shortest angle from start to end, in [-180, 180) degrees. */
    static double shortestAngleDeg(double startDeg, double endDeg) {
        return ((endDeg - startDeg) % 360.0 + 540.0) % 360.0 - 180.0;
    }

    static double dist(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** prefix[k] = distance along the path from point 0 to point k. */
    static double[] prefixDistances(List<PathPoint> points) {
        final int n = points.size();
        double[] prefix = new double[n];
        for (int i = 1; i < n; i++) {
            PathPoint a = points.get(i - 1);
            PathPoint b = points.get(i);
            prefix[i] = prefix[i - 1] + dist(a.x, a.y, b.x, b.y);
        }
        return prefix;
    }

    /** First index >= from that has a theta, or -1 if none. */
    static int nextHeadingIndex(List<PathPoint> points, int from) {
        for (int j = Math.max(0, from); j < points.size(); j++) {
            if (points.get(j).theta != null) return j;
        }
        return -1;
    }

    /** Last index < before that has a theta, or -1 if none. */
    static int prevHeadingIndex(List<PathPoint> points, int before) {
        for (int j = Math.min(before, points.size()) - 1; j >= 0; j--) {
            if (points.get(j).theta != null) return j;
        }
        return -1;
    }

    /** Point params merged on top of path params, with defaults filled in. {@code interpolate} is the exception: path OR point. */
    static final class ResolvedParams {
        /** 0 when unset */
        final double minSpeed;
        /** +inf when unset (caller clamps to the robot's max) */
        final double maxSpeed;
        /** +inf when unset (caller clamps to the robot's limit) */
        final double maxAccel;
        /** NaN when unset (no early exit) */
        final double earlyExitRange;
        /** +inf when unset */
        final double timeout;
        /** true if the path or the point turns it on (on for the path can't be turned off per point) */
        final boolean interpolate;
        /** the point's own (AUTO when unset); there's no path-wide one */
        final TurnDirection turnDirection;

        ResolvedParams(double minSpeed, double maxSpeed, double maxAccel, double earlyExitRange, double timeout, boolean interpolate,
                TurnDirection turnDirection) {
            this.minSpeed = minSpeed;
            this.maxSpeed = maxSpeed;
            this.maxAccel = maxAccel;
            this.earlyExitRange = earlyExitRange;
            this.timeout = timeout;
            this.interpolate = interpolate;
            this.turnDirection = turnDirection;
        }

        boolean hasEarlyExit() {
            return !Double.isNaN(earlyExitRange);
        }
    }

    private static Double pick(Double point, Double path) {
        return point != null ? point : path;
    }

    static ResolvedParams resolve(PathParams path, PathParams point) {
        PathParams a = path != null ? path : PathParams.EMPTY;
        PathParams b = point != null ? point : PathParams.EMPTY;
        Double min = pick(b.minSpeed, a.minSpeed);
        Double max = pick(b.maxSpeed, a.maxSpeed);
        Double accel = pick(b.maxAccel, a.maxAccel);
        Double exit = pick(b.earlyExitRange, a.earlyExitRange);
        Double timeout = pick(b.timeout, a.timeout);
        // interpolate on for the whole path turns it on for every point; otherwise each point decides (default off)
        boolean interp = Boolean.TRUE.equals(a.interpolate) || Boolean.TRUE.equals(b.interpolate);
        // turn direction is per point only: each turn picks its own way round
        TurnDirection turn = b.turnDirection != null ? b.turnDirection : TurnDirection.AUTO;
        return new ResolvedParams(
            min != null ? min : 0.0,
            max != null ? max : Double.POSITIVE_INFINITY,
            accel != null ? accel : Double.POSITIVE_INFINITY,
            exit != null ? exit : Double.NaN,
            timeout != null ? timeout : Double.POSITIVE_INFINITY,
            interp,
            turn);
    }

    /**
     * How far to turn (degrees) to get from heading {@code fromDeg} to {@code toDeg} going round {@code dir}:
     * AUTO the shorter way, in [-180, 180); CCW in [0, 360); CW in (-360, 0]. Already facing it = 0 either way.
     */
    static double turnDeltaDeg(double fromDeg, double toDeg, TurnDirection dir) {
        double d = shortestAngleDeg(fromDeg, toDeg);
        if (dir == TurnDirection.CCW && d < 0) return d + 360.0;
        if (dir == TurnDirection.CW && d > 0) return d - 360.0;
        return d;
    }

    static ResolvedParams[] resolveAll(PathDefinition def) {
        ResolvedParams[] out = new ResolvedParams[def.points.size()];
        for (int i = 0; i < out.length; i++) out[i] = resolve(def.params, def.points.get(i).params);
        return out;
    }

    /**
     * Clamp the raw controller speed to the point's [min, max].
     * The last point ignores minSpeed so the robot actually settles instead of orbiting the target.
     */
    static double clampSpeed(double rawSpeed, ResolvedParams p, boolean isLast, double robotMax) {
        double max = Math.min(p.maxSpeed, robotMax);
        double min = isLast ? 0.0 : Math.min(p.minSpeed, max);
        if (rawSpeed <= EPS) return 0.0;
        return clamp(rawSpeed, min, max);
    }

    /**
     * Fastest speed toward a point that can still slow down to {@code vEnd} by the time it gets there:
     * {@code sqrt(vEnd² + 2·0.9·a·(d − vNow·(dt + lag)))}. The 90% and the look-ahead keep it from
     * overshooting: steering and braking share the same acceleration budget, and the robot covers
     * {@code vNow·(dt + lag)} before a new command takes effect.
     *
     * @param d distance to where the speed has to be down to vEnd (m)
     * @param vNow current speed (m/s)
     * @param vEnd speed allowed there (0 = stop)
     * @param accel acceleration limit (m/s²)
     * @param dt loop period (s)
     * @param lag how long the drivetrain takes to follow a new velocity command (s)
     */
    static double brakingSpeed(double d, double vNow, double vEnd, double accel, double dt, double lag) {
        double dPlan = Math.max(0.0, d - vNow * (dt + lag));
        return Math.sqrt(vEnd * vEnd + 2.0 * BRAKE_MARGIN * accel * dPlan);
    }

    /** {@link #brakingSpeed(double, double, double, double, double, double)} for a drivetrain with no lag. */
    static double brakingSpeed(double d, double vNow, double vEnd, double accel, double dt) {
        return brakingSpeed(d, vNow, vEnd, accel, dt, 0.0);
    }

    /**
     * Fastest speed for turning at {@code p}, from the direction prev→p onto p→next, so the turn fits inside the
     * circle of radius {@code r} around p where the robot moves on to the next point. A turn of angle θ at the
     * acceleration limit is an arc of radius R = v²/a that starts R·tan(θ/2) before the corner, so it fits while
     * {@code v ≤ sqrt(0.9·a·r / tan(θ/2))}. Straight through: no limit. Doubling back, or a zero-length side: 0.
     *
     * <p>A drivetrain that takes {@code lag} seconds to respond starts the turn v·lag late, leaving r − v·lag to fit
     * it in: v² = k·(r − v·lag) with k = 0.9·a / tan(θ/2), so v = (√(k²·lag² + 4·k·r) − k·lag) / 2.
     */
    static double cornerSpeed(PathPoint prev, PathPoint p, PathPoint next, double r, double accel, double lag) {
        double ax = p.x - prev.x;
        double ay = p.y - prev.y;
        double bx = next.x - p.x;
        double by = next.y - p.y;
        double la = Math.hypot(ax, ay);
        double lb = Math.hypot(bx, by);
        if (la <= EPS || lb <= EPS) return 0.0;
        double theta = Math.acos(clamp((ax * bx + ay * by) / (la * lb), -1.0, 1.0));
        double t = Math.tan(theta / 2.0);
        if (t <= 1e-6) return Double.POSITIVE_INFINITY;
        double k = BRAKE_MARGIN * accel / t;
        return (Math.sqrt(k * k * lag * lag + 4.0 * k * r) - k * lag) / 2.0;
    }

    /**
     * The speed the robot may still have when it moves on from each point (m/s), so it doesn't brake to a stop at
     * every point it only passes through. 0 means stop there: the first and last points, and heading points without
     * an early exit range (they have to settle on their heading). Every other point gets the lowest of:
     * <ul>
     *   <li>its corner ({@link #cornerSpeed}), turning inside its early exit range, or the position tolerance
     *       if it has none, allowing for the drivetrain's response lag;</li>
     *   <li>the next point: slow enough to still get down to that point's speed in the room between them;</li>
     *   <li>this point's and the next point's max speed.</li>
     * </ul>
     * minSpeed isn't part of this; it still floors the planned speed as before.
     */
    static double[] passSpeeds(List<PathPoint> pts, ResolvedParams[] params, double robotMax, double robotAccel,
            double posTolerance, double lag) {
        final int n = pts.size();
        double[] pass = new double[n];
        for (int k = n - 2; k >= 1; k--) {
            PathPoint p = pts.get(k);
            PathPoint next = pts.get(k + 1);
            ResolvedParams here = params[k];
            ResolvedParams after = params[k + 1];
            if (p.theta != null && !here.hasEarlyExit()) continue; // stop and turn
            double accel = Math.min(Math.min(here.maxAccel, after.maxAccel), robotAccel);
            double r = here.hasEarlyExit() ? here.earlyExitRange : posTolerance;
            double rNext = k + 1 < n - 1 && after.hasEarlyExit() ? after.earlyExitRange : 0.0;
            double room = Math.max(0.0, dist(p.x, p.y, next.x, next.y) - r - rNext);
            double reach = Math.sqrt(pass[k + 1] * pass[k + 1] + 2.0 * BRAKE_MARGIN * accel * room);
            double cap = Math.min(Math.min(here.maxSpeed, after.maxSpeed), robotMax);
            pass[k] = Math.min(Math.min(cornerSpeed(pts.get(k - 1), p, next, r, accel, lag), reach), cap);
        }
        return pass;
    }

    /**
     * Move velocity (vx, vy) toward (tx, ty), changing it by at most {@code maxDelta} (m/s) as a vector.
     * Speeding up, braking and turning all share this one limit, which is what rounds corners off.
     *
     * @return the new {vx, vy}
     */
    static double[] slew(double vx, double vy, double tx, double ty, double maxDelta) {
        double dx = tx - vx;
        double dy = ty - vy;
        double m = Math.hypot(dx, dy);
        if (m <= maxDelta || m <= EPS) return new double[] {tx, ty};
        return new double[] {vx + dx / m * maxDelta, vy + dy / m * maxDelta};
    }

    /**
     * Heading (degrees) to face while driving toward points[k].
     *
     * <p>interpolate on heading point B = turn from the previous heading point A to B in proportion to the distance
     * covered (no snapping); see {@link #turnCurve}. interpolate off on B = aim for B as soon as the robot leaves A
     * (turning as fast as the heading controller goes). Before the first heading point, "A" is wherever the robot was
     * facing when the path started.
     */
    static double desiredHeadingDeg(List<PathPoint> pts, double[] prefix, ResolvedParams[] params, int k,
            double poseX, double poseY, double startX, double startY, double startHeadingDeg) {
        return headingCurve(pts, prefix, params, k, poseX, poseY, startX, startY, startHeadingDeg)[0];
    }

    /**
     * How fast the heading from {@link #desiredHeadingDeg} is turning (deg/s) while the robot drives toward points[k]
     * at (vx, vy): the heading controller's feedforward, so it doesn't lag behind a turn spread along the path.
     * 0 while holding a heading, or aiming straight for the next one without interpolation.
     */
    static double desiredHeadingRateDegPerSec(List<PathPoint> pts, double[] prefix, ResolvedParams[] params, int k,
            double poseX, double poseY, double vx, double vy, double startX, double startY, double startHeadingDeg) {
        double slope = headingCurve(pts, prefix, params, k, poseX, poseY, startX, startY, startHeadingDeg)[1];
        if (slope == 0.0) return 0.0;
        PathPoint target = pts.get(k);
        double d = dist(poseX, poseY, target.x, target.y);
        if (d <= EPS) return 0.0;
        // the robot moves along the path as fast as it closes on its target
        double closing = ((target.x - poseX) * vx + (target.y - poseY) * vy) / d;
        return slope * closing;
    }

    /** {heading (deg), its rate along the path (deg/m)} while driving toward points[k]. */
    private static double[] headingCurve(List<PathPoint> pts, double[] prefix, ResolvedParams[] params, int k,
            double poseX, double poseY, double startX, double startY, double startHeadingDeg) {
        int a = prevHeadingIndex(pts, k);
        int b = nextHeadingIndex(pts, k);
        if (b < 0) return new double[] {a >= 0 ? pts.get(a).theta : startHeadingDeg, 0.0}; // nothing ahead, hold what we had
        if (!params[b].interpolate) return new double[] {pts.get(b).theta, 0.0};
        // how far along the path the robot is: where its target is, less how far it still has to go to it
        double s = prefix[k] - dist(poseX, poseY, pts.get(k).x, pts.get(k).y);
        return turnCurve(pts, prefix, params, a, b, s, dist(startX, startY, pts.get(0).x, pts.get(0).y), startHeadingDeg);
    }

    /**
     * {heading, deg/m} at path position {@code s} (m, 0 at point 0) on the turn from heading point a (-1: the robot's
     * start, {@code startDist} before point 0) to heading point b.
     *
     * <p>The turn is proportional to distance: it runs from A to B along the path, and x% of the way there it has
     * turned x% of the way from A's heading to B's, so the robot faces each heading at its point. Which turn it's on
     * goes by how far along the path the robot is, not by which point it's driving to: when it moves on from B early (at
     * B's early exit range) it's still short of B, so it keeps turning into B until it's level with it, then carries
     * straight on into the next turn. Turns are chained back to back, so the heading never sits still or jumps between
     * them, and since the rate per meter is constant over a stretch, the spin rate follows the robot's speed.
     */
    private static double[] turnCurve(List<PathPoint> pts, double[] prefix, ResolvedParams[] params, int a, int b,
            double s, double startDist, double startHeadingDeg) {
        double s0 = a >= 0 ? prefix[a] : -startDist;
        double s1 = prefix[b];
        // moved on from A early but not level with it yet: still on the turn into A (if that one is interpolated)
        if (s < s0 && a >= 0 && params[a].interpolate) {
            return turnCurve(pts, prefix, params, prevHeadingIndex(pts, a), a, s, startDist, startHeadingDeg);
        }
        double y0 = a >= 0 ? pts.get(a).theta : startHeadingDeg;
        double y1 = y0 + turnDeltaDeg(y0, pts.get(b).theta, params[b].turnDirection);
        int b2 = nextHeadingIndex(pts, b + 1);
        // right at B: carry on into the next turn, so its rate doesn't drop
        if (s >= s1 && b2 >= 0 && params[b2].interpolate) return turnCurve(pts, prefix, params, b, b2, s, startDist, startHeadingDeg);
        double h = s1 - s0;
        if (h <= EPS || s >= s1) return new double[] {y1, 0.0};
        if (s < s0) return new double[] {y0, 0.0};
        double rate = (y1 - y0) / h;
        return new double[] {y0 + rate * (s - s0), rate};
    }

    /**
     * Is the robot done with this point? Timeout passed, inside early exit range (never on the last point),
     * or within the position tolerance of it (straight-line distance; plus heading tolerance for heading points).
     * Translation-only points don't check heading, since the robot may be mid-turn on purpose.
     */
    static boolean isPointFinished(PathPoint p, ResolvedParams params, boolean isLast,
            double poseX, double poseY, double headingDeg, double elapsed,
            double posTolerance, double headingToleranceDeg) {
        if (elapsed > params.timeout) return true;

        double d = dist(poseX, poseY, p.x, p.y);
        if (!isLast && params.hasEarlyExit() && d < params.earlyExitRange) return true;

        if (d >= posTolerance) return false;
        if (p.theta == null) return true;
        return Math.abs(shortestAngleDeg(headingDeg, p.theta)) < headingToleranceDeg;
    }

    /** How far (0..1) (x, y) is along segment a -> b, by projecting onto it. Used for events. */
    static double segmentProgress(PathPoint a, PathPoint b, double x, double y) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double len2 = dx * dx + dy * dy;
        if (len2 <= EPS) return 1.0;
        return clamp(((x - a.x) * dx + (y - a.y) * dy) / len2, 0.0, 1.0);
    }
}
