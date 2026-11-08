package pointweaver;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;

import java.nio.file.Path;

import pointweaver.core.FollowerSettings;
import pointweaver.core.PathLoadException;
import pointweaver.core.RobotSettings;
import pointweaver.core.RobotSettingsParser;

/**
 * Robot limits and tuning for PointWeaver. Usually loaded from {@code deploy/pointweaver/robot.json}
 * (edit it in the PointWeaver app's Robot panel), so the robot and the app's ghost use the same numbers:
 *
 * <pre>{@code
 * PointWeaver.configure(drivetrain);                                    // robot.json as-is
 * PointWeaver.configure(drivetrain, PointWeaverConfig.fromDeploy()      // robot.json, with code overrides
 *     .headingPID(3.0, 0, 0).build());
 * PointWeaver.configure(drivetrain, PointWeaverConfig.builder(TunerConstants.kSpeedAt12Volts).build()); // no file
 * }</pre>
 */
public final class PointWeaverConfig {
    /** Deploy sub-folder with robot.json and paths/. */
    public static final String DEFAULT_DEPLOY_FOLDER = "pointweaver";

    final double maxSpeed;
    final double maxAcceleration;
    final double translationKp;
    final double translationKi;
    final double translationKd;
    final double headingKp;
    final double headingKi;
    final double headingKd;
    final double positionTolerance;
    final double headingToleranceDeg;
    final double period;
    final String deployFolder;
    final boolean telemetry;
    final String telemetryTable;

    private PointWeaverConfig(Builder b) {
        maxSpeed = b.maxSpeed;
        maxAcceleration = b.maxAcceleration;
        translationKp = b.translationKp;
        translationKi = b.translationKi;
        translationKd = b.translationKd;
        headingKp = b.headingKp;
        headingKi = b.headingKi;
        headingKd = b.headingKd;
        positionTolerance = b.positionTolerance;
        headingToleranceDeg = b.headingToleranceDeg;
        period = b.period;
        deployFolder = b.deployFolder;
        telemetry = b.telemetry;
        telemetryTable = b.telemetryTable;
    }

    /** @param maxSpeed the drivetrain's top speed, e.g. {@code TunerConstants.kSpeedAt12Volts} */
    public static Builder builder(LinearVelocity maxSpeed) {
        return builder(maxSpeed.in(MetersPerSecond));
    }

    /** @param maxSpeedMetersPerSecond the drivetrain's top speed (m/s) */
    public static Builder builder(double maxSpeedMetersPerSecond) {
        return new Builder(maxSpeedMetersPerSecond);
    }

    /**
     * A builder pre-filled from {@code deploy/pointweaver/robot.json}.
     *
     * @throws PathLoadException if robot.json is missing, invalid, or has no maxSpeed
     */
    public static Builder fromDeploy() {
        return fromDeploy(DEFAULT_DEPLOY_FOLDER);
    }

    /**
     * A builder pre-filled from {@code deploy/<folder>/robot.json}.
     *
     * @throws PathLoadException if robot.json is missing, invalid, or has no maxSpeed
     */
    public static Builder fromDeploy(String folder) {
        Path file = Filesystem.getDeployDirectory().toPath().resolve(folder).resolve("robot.json");
        return fromSettings(RobotSettingsParser.parse(file)).deployFolder(folder);
    }

    /**
     * A builder pre-filled from already parsed robot settings.
     *
     * @throws PathLoadException if the settings have no maxSpeed
     */
    public static Builder fromSettings(RobotSettings s) {
        if (s.maxSpeed == null) throw new PathLoadException("robot.json: 'maxSpeed' is required");
        Builder b = builder(s.maxSpeed);
        if (s.maxAccel != null) b.maxAcceleration(s.maxAccel);
        if (s.translationPID != null) b.translationPID(s.translationPID[0], s.translationPID[1], s.translationPID[2]);
        if (s.headingPID != null) b.headingPID(s.headingPID[0], s.headingPID[1], s.headingPID[2]);
        if (s.positionTolerance != null) b.positionTolerance(s.positionTolerance);
        if (s.headingTolerance != null) b.headingTolerance(s.headingTolerance);
        return b;
    }

    FollowerSettings toSettings() {
        return new FollowerSettings(maxSpeed, maxAcceleration, translationKp, translationKi, translationKd,
            positionTolerance, headingToleranceDeg, period);
    }

    public static final class Builder {
        private final double maxSpeed;
        private double maxAcceleration = 3.0;
        private double translationKp = 5.0;
        private double translationKi = 0.0;
        private double translationKd = 0.0;
        private double headingKp = 2.0;
        private double headingKi = 0.0;
        private double headingKd = 0.0;
        private double positionTolerance = 0.05;
        private double headingToleranceDeg = 2.0;
        private double period = 0.02;
        private String deployFolder = DEFAULT_DEPLOY_FOLDER;
        private boolean telemetry = true;
        private String telemetryTable = "PointWeaver";

        private Builder(double maxSpeed) {
            this.maxSpeed = maxSpeed;
        }

        /** Acceleration limit for speeding up, braking and turning corners (m/s²). Default 3. */
        public Builder maxAcceleration(double metersPerSecondSquared) {
            maxAcceleration = metersPerSecondSquared;
            return this;
        }

        /** Final-approach controller on distance to the point (m/s per m). Default 5, 0, 0. */
        public Builder translationPID(double kP, double kI, double kD) {
            translationKp = kP;
            translationKi = kI;
            translationKd = kD;
            return this;
        }

        /** CTRE heading controller (rad/s per rad of error). Default 2, 0, 0. */
        public Builder headingPID(double kP, double kI, double kD) {
            headingKp = kP;
            headingKi = kI;
            headingKd = kD;
            return this;
        }

        /** A point counts as reached inside this box, per axis (m). Default 0.05. */
        public Builder positionTolerance(double meters) {
            positionTolerance = meters;
            return this;
        }

        /** A heading point counts as reached inside this (deg). Default 2. */
        public Builder headingTolerance(double degrees) {
            headingToleranceDeg = degrees;
            return this;
        }

        /** Robot loop period (s). Default 0.02. */
        public Builder period(double seconds) {
            period = seconds;
            return this;
        }

        /** Deploy sub-folder holding robot.json and paths/. Default "pointweaver". */
        public Builder deployFolder(String folder) {
            deployFolder = folder;
            return this;
        }

        /** Publish follower state to NetworkTables. Default on. */
        public Builder telemetry(boolean enabled) {
            telemetry = enabled;
            return this;
        }

        /** NetworkTables table for telemetry. Default "PointWeaver". */
        public Builder telemetryTable(String table) {
            telemetryTable = table;
            return this;
        }

        public PointWeaverConfig build() {
            PointWeaverConfig c = new PointWeaverConfig(this);
            c.toSettings(); // validates
            return c;
        }
    }
}
