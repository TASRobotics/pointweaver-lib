package pointweaver.core;

/**
 * Contents of {@code deploy/pointweaver/robot.json}, the robot settings shared by the library and the
 * PointWeaver app's ghost. Every field is optional ({@code null} = use the library default).
 */
public final class RobotSettings {
    /** Top speed (m/s), e.g. TunerConstants.kSpeedAt12Volts. */
    public final Double maxSpeed;
    /** Acceleration limit (m/s²). */
    public final Double maxAccel;
    /** {kP, kI, kD} for distance-to-point, or null. */
    public final double[] translationPID;
    /** {kP, kI, kD} for CTRE's heading controller, or null. */
    public final double[] headingPID;
    /** Point reached within this distance of it (m). */
    public final Double positionTolerance;
    /** Heading point reached inside this (deg). */
    public final Double headingTolerance;
    /** Fastest the robot can spin (deg/s). */
    public final Double maxAngularRate;
    /** Bumper to bumper (m). Only the app's ghost uses these. */
    public final Double robotLength;
    public final Double robotWidth;
    /** How long the drivetrain takes to follow a new velocity command (s). */
    public final Double responseLag;

    public RobotSettings(Double maxSpeed, Double maxAccel, double[] translationPID, double[] headingPID,
            Double positionTolerance, Double headingTolerance, Double maxAngularRate, Double robotLength, Double robotWidth) {
        this(maxSpeed, maxAccel, translationPID, headingPID, positionTolerance, headingTolerance, maxAngularRate,
            robotLength, robotWidth, null);
    }

    public RobotSettings(Double maxSpeed, Double maxAccel, double[] translationPID, double[] headingPID,
            Double positionTolerance, Double headingTolerance, Double maxAngularRate, Double robotLength, Double robotWidth,
            Double responseLag) {
        this.maxSpeed = maxSpeed;
        this.maxAccel = maxAccel;
        this.translationPID = translationPID != null ? translationPID.clone() : null;
        this.headingPID = headingPID != null ? headingPID.clone() : null;
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
        this.maxAngularRate = maxAngularRate;
        this.robotLength = robotLength;
        this.robotWidth = robotWidth;
        this.responseLag = responseLag;
    }
}
