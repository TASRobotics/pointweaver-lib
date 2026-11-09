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

    /** smoothstep easing, clamped to [0, 1]. */
    static double smoothstep(double x) {
        if (x <= 0) return 0.0;
        if (x >= 1) return 1.0;
        return x * x * (3 - 2 * x);
    }

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

    /** Point params merged on top of path params, with defaults filled in. */
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
        /** false when unset */
        final boolean interpolate;

        ResolvedParams(double minSpeed, double maxSpeed, double maxAccel, double earlyExitRange, double timeout, boolean interpolate) {
            this.minSpeed = minSpeed;
            this.maxSpeed = maxSpeed;
            this.maxAccel = maxAccel;
            this.earlyExitRange = earlyExitRange;
            this.timeout = timeout;
            this.interpolate = interpolate;
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
        Boolean interp = b.interpolate != null ? b.interpolate : a.interpolate;
        return new ResolvedParams(
            min != null ? min : 0.0,
            max != null ? max : Double.POSITIVE_INFINITY,
            accel != null ? accel : Double.POSITIVE_INFINITY,
            exit != null ? exit : Double.NaN,
            timeout != null ? timeout : Double.POSITIVE_INFINITY,
            interp != null && interp);
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
     * {@code sqrt(vEnd² + 2·0.9·a·(d − vNow·dt))}. The 90% and the one-loop look-ahead keep it from
     * overshooting, since steering and braking share the same acceleration budget.
     *
     * @param d distance to the point (m)
     * @param vNow current commanded speed (m/s)
     * @param vEnd speed allowed at the point (0 = stop there)
     * @param accel acceleration limit (m/s²)
     * @param dt loop period (s)
     */
    static double brakingSpeed(double d, double vNow, double vEnd, double accel, double dt) {
        double dPlan = Math.max(0.0, d - vNow * dt);
        return Math.sqrt(vEnd * vEnd + 2.0 * BRAKE_MARGIN * accel * dPlan);
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
     * <p>Every [point, trans, ..., point] sandwich goes from heading A to heading B.
     * interpolate on B = smoothly turn from A to B over the whole sandwich (no snapping).
     * interpolate off on B = hold A on the translation points, then snap to B on the last leg.
     * Before the first heading point, "A" is wherever the robot was facing when the path started.
     */
    static double desiredHeadingDeg(List<PathPoint> pts, double[] prefix, ResolvedParams[] params, int k,
            double poseX, double poseY, double startX, double startY, double startHeadingDeg) {
        int a = prevHeadingIndex(pts, k);
        int b = nextHeadingIndex(pts, k);
        double fromDeg = a >= 0 ? pts.get(a).theta : startHeadingDeg;

        if (b < 0) return fromDeg; // nothing ahead, hold what we had
        double toDeg = pts.get(b).theta;

        if (!params[b].interpolate) {
            return k == b ? toDeg : fromDeg;
        }

        PathPoint target = pts.get(k);
        // whole sandwich length, and how much of it is left (along the path, not straight-line)
        double total = a >= 0
            ? prefix[b] - prefix[a]
            : dist(startX, startY, pts.get(0).x, pts.get(0).y) + prefix[b] - prefix[0];
        double remaining = dist(poseX, poseY, target.x, target.y) + prefix[b] - prefix[k];

        if (total <= EPS) return toDeg;
        double progress = smoothstep(clamp(1.0 - remaining / total, 0.0, 1.0));
        return fromDeg + shortestAngleDeg(fromDeg, toDeg) * progress;
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
