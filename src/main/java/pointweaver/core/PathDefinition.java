package pointweaver.core;

import java.util.List;

/** A parsed path file. Immutable, so one loaded path can be shared by any number of followers. */
public final class PathDefinition {
    /** Name of the file (without .json), or "" when parsed from text. */
    public final String name;
    public final List<PathPoint> points;
    /** Path-wide defaults (never null; {@link PathParams#EMPTY} when none). */
    public final PathParams params;
    /** Reset odometry to the first point when the path starts. */
    public final boolean resetPose;

    public PathDefinition(String name, List<PathPoint> points, PathParams params, boolean resetPose) {
        this.name = name != null ? name : "";
        this.points = points != null ? List.copyOf(points) : List.of();
        this.params = params != null ? params : PathParams.EMPTY;
        this.resetPose = resetPose;
    }

    /** A path with no points. Following it finishes immediately. */
    public static PathDefinition empty(String name) {
        return new PathDefinition(name, List.of(), null, false);
    }
}
