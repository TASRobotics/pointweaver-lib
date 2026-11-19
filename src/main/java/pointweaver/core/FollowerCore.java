package pointweaver.core;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import pointweaver.core.PathMath.ResolvedParams;

/**
 * The path follower itself, as plain Java: feed it the robot pose every loop, get back field-relative
 * velocities and a target heading. No WPILib or CTRE types, so it runs the same on any robot framework
 * (TimedRobot, command-based, 2027 OpModes) and in unit tests.
 *
 * <p>Each point is driven to in a straight line, in order. Points the robot only passes through get a speed
 * it may carry past them ({@link PathMath#passSpeeds}: limited by the corner, the next point and max speed),
 * and the robot drives at the fastest speed that can still slow to that by the time it moves on. Points it
 * stops at (the last one, heading points without an early exit range, sharp turnarounds) use the slower of
 * the translation PID (final approach) and a braking curve that can still stop there. Either way the speed is
 * clamped to the point's min/max speed, and the commanded velocity changes by at most {@code maxAccel·dt} per
 * loop, as a vector, so starting, stopping and corners are all acceleration-limited.
 *
 * <p>A point finishes on its timeout, inside its early exit range (not the last point), or inside the
 * position tolerance (plus heading tolerance for heading points).
 *
 * <p>Usage: {@link #resetPose} (optional) → {@link #start} → {@link #update} every loop until
 * {@link Output#finished}.
 */
public final class FollowerCore {
    /** A pose in field coordinates, heading in degrees. */
    public static final class Pose {
        public final double x;
        public final double y;
        public final double headingDeg;

        public Pose(double x, double y, double headingDeg) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }
    }

    /** What to do this loop. Velocities are field-relative (blue origin), m/s. */
    public static final class Output {
        public final double vx;
        public final double vy;
        /** Heading to point the robot at (deg, CCW from +x). */
        public final double headingDeg;
        /** How fast that heading is turning (deg/s, CCW+): feedforward for the heading controller. */
        public final double headingRateDegPerSec;
        /** Index of the point being driven to (the last index once finished). */
        public final int targetIndex;
        /** Straight-line distance to that point (m). */
        public final double distance;
        /** Speed the planner wanted this loop, before the acceleration limit (m/s). */
        public final double plannedSpeed;
        /** The path is done; command zero speed. */
        public final boolean finished;

        Output(double vx, double vy, double headingDeg, double headingRateDegPerSec, int targetIndex, double distance,
                double plannedSpeed, boolean finished) {
            this.vx = vx;
            this.vy = vy;
            this.headingDeg = headingDeg;
            this.headingRateDegPerSec = headingRateDegPerSec;
            this.targetIndex = targetIndex;
            this.distance = distance;
            this.plannedSpeed = plannedSpeed;
            this.finished = finished;
        }

        /** Commanded speed (m/s). */
        public double speed() {
            return Math.hypot(vx, vy);
        }
    }

    private final PathDefinition path;
    private final FollowerSettings settings;
    private final Consumer<String> events;
    private final List<PathPoint> pts;
    private final double[] prefix;
    private final ResolvedParams[] params;
    /** speed the robot may still have when it moves on from each point (0 = stop there) */
    private final double[] passSpeeds;
    private final boolean[][] fired;
    private final Pid pid;

    private int index;
    private double pointStartTime;
    private double startX;
    private double startY;
    private double startHeadingDeg;
    /** Commanded velocity, carried across points so corners are acceleration-limited too. */
    private double cmdVx;
    private double cmdVy;
    private boolean started;
    private boolean finished;

    /**
     * @param path the path to follow
     * @param settings robot limits and tuning
     * @param events called with an event's name when it fires (may be null)
     */
    public FollowerCore(PathDefinition path, FollowerSettings settings, Consumer<String> events) {
        this.path = path;
        this.settings = settings;
        this.events = events != null ? events : name -> {};
        this.pts = path.points;
        this.prefix = PathMath.prefixDistances(pts);
        this.params = PathMath.resolveAll(path);
        this.passSpeeds = PathMath.passSpeeds(pts, params, settings.maxSpeed, settings.maxAcceleration, settings.positionTolerance,
            settings.responseLag);
        this.fired = new boolean[pts.size()][];
        for (int i = 0; i < pts.size(); i++) fired[i] = new boolean[pts.get(i).events.size()];
        this.pid = new Pid(settings.translationKp, settings.translationKi, settings.translationKd, settings.period);
    }

    public PathDefinition path() {
        return path;
    }

    /**
     * If the path wants odometry reset, the pose to reset to: point 0, with its heading,
     * or the robot's current heading if point 0 has none. Otherwise null.
     */
    public Pose resetPose(double currentHeadingDeg) {
        if (!path.resetPose || pts.isEmpty()) return null;
        PathPoint first = pts.get(0);
        return new Pose(first.x, first.y, first.theta != null ? first.theta : currentHeadingDeg);
    }

    /** Begin (or restart) the path from a robot at rest. */
    public void start(double x, double y, double headingDeg, double timeSec) {
        start(x, y, headingDeg, 0, 0, timeSec);
    }

    /**
     * Begin (or restart) the path from the robot's current pose and field-relative velocity,
     * so a robot that's already moving doesn't get jerked to a stop.
     */
    public void start(double x, double y, double headingDeg, double vx, double vy, double timeSec) {
        startX = x;
        startY = y;
        startHeadingDeg = headingDeg;
        cmdVx = vx;
        cmdVy = vy;
        for (boolean[] f : fired) Arrays.fill(f, false);
        index = 0;
        pointStartTime = timeSec;
        pid.reset();
        started = true;
        finished = pts.isEmpty();
    }

    /**
     * One loop, when the robot's measured velocity isn't available: the last command stands in for it.
     * Call {@link #start} first.
     */
    public Output update(double x, double y, double headingDeg, double timeSec) {
        return update(x, y, headingDeg, cmdVx, cmdVy, timeSec);
    }

    /**
     * One loop: where to drive this period. Call {@link #start} first.
     *
     * @param measuredVx the robot's measured field-relative velocity (m/s), used for braking
     * @param measuredVy see measuredVx
     */
    public Output update(double x, double y, double headingDeg, double measuredVx, double measuredVy, double timeSec) {
        if (!started) throw new IllegalStateException("call start() before update()");
        if (finished) return stopped(headingDeg);

        PathPoint target = pts.get(index);
        ResolvedParams prm = params[index];
        boolean isLast = index == pts.size() - 1;
        double accel = Math.min(prm.maxAccel, settings.maxAcceleration);

        double xError = target.x - x;
        double yError = target.y - y;
        double distance = Math.hypot(xError, yError);
        double measuredSpeed = Math.hypot(measuredVx, measuredVy);

        double approach = Math.abs(pid.calculate(distance, 0));
        double passSpeed = passSpeeds[index];
        double planned;
        if (passSpeed > PathMath.EPS) {
            // passing through: fastest speed that's down to the pass speed by the time the robot moves on
            double moveOn = prm.hasEarlyExit() ? prm.earlyExitRange : 0.0;
            planned = PathMath.brakingSpeed(distance - moveOn, measuredSpeed, passSpeed, accel, settings.period, settings.responseLag);
        } else {
            // stopping here: slower of the final-approach PID and the fastest speed that can still stop at the point,
            // leaving room to cancel any sideways velocity first (the straight-line braking curve can't see it)
            double sideways = distance > PathMath.EPS ? Math.abs(xError * measuredVy - yError * measuredVx) / distance : 0.0;
            double room = distance - sideways * sideways / (2.0 * accel);
            double braking = PathMath.brakingSpeed(room, measuredSpeed, 0.0, accel, settings.period, settings.responseLag);
            planned = Math.min(approach, braking);
        }
        planned = PathMath.clampSpeed(planned, prm, isLast, settings.maxSpeed);
        double tx = 0;
        double ty = 0;
        if (distance > PathMath.EPS) {
            tx = xError / distance * planned;
            ty = yError / distance * planned;
        }
        double[] v = PathMath.slew(cmdVx, cmdVy, tx, ty, accel * settings.period);
        cmdVx = v[0];
        cmdVy = v[1];

        double wantDeg = PathMath.desiredHeadingDeg(pts, prefix, params, index, x, y, startX, startY, startHeadingDeg);
        double wantRate = PathMath.desiredHeadingRateDegPerSec(pts, prefix, params, index, x, y, cmdVx, cmdVy,
            startX, startY, startHeadingDeg);

        if (index > 0) fireEvents(index - 1, PathMath.segmentProgress(pts.get(index - 1), target, x, y));

        boolean done = PathMath.isPointFinished(target, prm, isLast, x, y, headingDeg, timeSec - pointStartTime,
            settings.positionTolerance, settings.headingToleranceDeg);
        if (!done) return new Output(cmdVx, cmdVy, wantDeg, wantRate, index, distance, planned, false);

        // point finished: anything early exit skipped over still fires
        if (index > 0) fireEvents(index - 1, 1.0);
        if (isLast) {
            fireEvents(index, 1.0);
            finished = true;
            cmdVx = 0;
            cmdVy = 0;
            return new Output(0, 0, wantDeg, 0, index, distance, 0, true);
        }
        index++;
        pointStartTime = timeSec;
        pid.reset();
        return new Output(cmdVx, cmdVy, wantDeg, wantRate, index - 1, distance, planned, false);
    }

    /** Stop early (e.g. auto ended). Further updates return zero speed. */
    public void cancel() {
        finished = true;
        cmdVx = 0;
        cmdVy = 0;
    }

    public boolean isFinished() {
        return finished;
    }

    /** Index of the point currently being driven to. */
    public int targetIndex() {
        return index;
    }

    private Output stopped(double headingDeg) {
        int last = Math.max(0, Math.min(index, pts.size() - 1));
        return new Output(0, 0, headingDeg, 0, last, 0, 0, true);
    }

    private void fireEvents(int pointIdx, double progress) {
        List<PathEvent> evs = pts.get(pointIdx).events;
        for (int e = 0; e < evs.size(); e++) {
            PathEvent ev = evs.get(e);
            if (fired[pointIdx][e] || progress < ev.fraction) continue;
            fired[pointIdx][e] = true;
            events.accept(ev.name);
        }
    }
}
