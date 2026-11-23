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
    /** Acceleration limit (m/s²) toward this point, capped by the robot's limit. */
    public final Double maxAccel;
    /** Move on to the next point once this close (m). Ignored on the last point. */
    public final Double earlyExitRange;
    /** Give up on a point after this many seconds. No timeout = wait forever. */
    public final Double timeout;
    /**
     * On a heading point: turn smoothly into this heading across the stretch, instead of turning to it right away.
     * On the path, it turns this on for every heading point (a point can't turn it back off).
     */
    public final Boolean interpolate;
    /** On a heading point: which way an interpolated turn into it goes round (unset = AUTO). Ignored on the path. */
    public final TurnDirection turnDirection;

    public static final PathParams EMPTY = new PathParams(null, null, null, null, null, null, null);

    public PathParams(Double minSpeed, Double maxSpeed, Double maxAccel, Double earlyExitRange, Double timeout, Boolean interpolate) {
        this(minSpeed, maxSpeed, maxAccel, earlyExitRange, timeout, interpolate, null);
    }

    public PathParams(Double minSpeed, Double maxSpeed, Double maxAccel, Double earlyExitRange, Double timeout, Boolean interpolate,
            TurnDirection turnDirection) {
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
        this.maxAccel = maxAccel;
        this.earlyExitRange = earlyExitRange;
        this.timeout = timeout;
        this.interpolate = interpolate;
        this.turnDirection = turnDirection;
    }
}
