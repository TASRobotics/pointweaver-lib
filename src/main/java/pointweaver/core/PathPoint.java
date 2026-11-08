package pointweaver.core;

import java.util.List;

/**
 * One point of a path, in field coordinates (meters, blue alliance origin).
 * A point without {@link #theta} is translation-only: the robot passes through it
 * while holding or interpolating its heading.
 */
public final class PathPoint {
    public final double x;
    public final double y;
    /** Heading in degrees (CCW from +x), or {@code null} for a translation-only point. */
    public final Double theta;
    /** Per-point overrides (never null; {@link PathParams#EMPTY} when none). */
    public final PathParams params;
    /** Events attached to this point (never null). */
    public final List<PathEvent> events;

    public PathPoint(double x, double y, Double theta, PathParams params, List<PathEvent> events) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.params = params != null ? params : PathParams.EMPTY;
        this.events = events != null ? List.copyOf(events) : List.of();
    }
}
