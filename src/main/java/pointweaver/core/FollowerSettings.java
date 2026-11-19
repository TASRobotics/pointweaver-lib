package pointweaver.core;

/** The numbers {@link FollowerCore} needs. Built from {@code PointWeaverConfig} on the robot. */
public final class FollowerSettings {
    /** Robot's top speed (m/s). Paths can only go slower. */
    public final double maxSpeed;
    /** Acceleration limit on the velocity vector (m/s²). Paths can only go gentler. */
    public final double maxAcceleration;
    public final double translationKp;
    public final double translationKi;
    public final double translationKd;
    /** A point counts as reached within this distance of it (m). */
    public final double positionTolerance;
    /** A heading point counts as reached inside this (deg). */
    public final double headingToleranceDeg;
    /** Robot loop period (s). */
    public final double period;
    /** How long the drivetrain takes to follow a new velocity command (s); braking starts that much earlier. */
    public final double responseLag;

    /** Settings for a drivetrain with no response lag. */
    public FollowerSettings(double maxSpeed, double maxAcceleration, double translationKp, double translationKi,
            double translationKd, double positionTolerance, double headingToleranceDeg, double period) {
        this(maxSpeed, maxAcceleration, translationKp, translationKi, translationKd, positionTolerance,
            headingToleranceDeg, period, 0.0);
    }

    public FollowerSettings(double maxSpeed, double maxAcceleration, double translationKp, double translationKi,
            double translationKd, double positionTolerance, double headingToleranceDeg, double period, double responseLag) {
        if (!(maxSpeed > 0)) throw new IllegalArgumentException("maxSpeed must be > 0");
        if (!(maxAcceleration > 0)) throw new IllegalArgumentException("maxAcceleration must be > 0");
        if (!(period > 0)) throw new IllegalArgumentException("period must be > 0");
        if (!(responseLag >= 0)) throw new IllegalArgumentException("responseLag must be >= 0");
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.translationKp = translationKp;
        this.translationKi = translationKi;
        this.translationKd = translationKd;
        this.positionTolerance = positionTolerance;
        this.headingToleranceDeg = headingToleranceDeg;
        this.period = period;
        this.responseLag = responseLag;
    }
}
