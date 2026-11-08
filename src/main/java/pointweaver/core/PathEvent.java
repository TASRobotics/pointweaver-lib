package pointweaver.core;

/**
 * A named event on a point. Fires once the robot is {@code fraction} (0..1) of the way from this point
 * to the next one. Events on the last point fire when the path finishes.
 */
public final class PathEvent {
    public final String name;
    public final double fraction;

    public PathEvent(String name, double fraction) {
        this.name = name;
        this.fraction = fraction;
    }
}
