package pointweaver.core;

/**
 * Optional tuning for a whole path or a single point. A {@code null} field means "not set":
 * a point inherits the path's value, and the path falls back to the defaults in {@link PathMath#resolve}.
 */
public final class PathParams {
    /** Minimum translation speed (m/s). Ignored on the last point. */
    public final Double minSpeed;
    /** Maximum translation speed (m/s), capped by the robot's max speed. */
    public final Double maxSpeed;
    /** Move on to the next point once this close (m). Ignored on the last point. */
    public final Double earlyExitRange;
    /** Give up on a point after this many seconds. No timeout = wait forever. */
    public final Double timeout;
    /** On a heading point: turn smoothly into this heading instead of snapping on the last leg. */
    public final Boolean interpolate;

    public static final PathParams EMPTY = new PathParams(null, null, null, null, null);

    public PathParams(Double minSpeed, Double maxSpeed, Double earlyExitRange, Double timeout, Boolean interpolate) {
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
        this.earlyExitRange = earlyExitRange;
        this.timeout = timeout;
        this.interpolate = interpolate;
    }
}
