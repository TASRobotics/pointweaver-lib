package pointweaver;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import pointweaver.core.FollowerSettings;

/**
 * Robot limits and tuning for PointWeaver. Only the max speed is required:
 *
 * <pre>{@code
 * PointWeaverConfig.builder(TunerConstants.kSpeedAt12Volts)
 *     .translationPID(1.0, 0, 0)
 *     .headingPID(2.0, 0, 0)
 *     .build();
 * }</pre>
 */
public final class PointWeaverConfig {
    final double maxSpeed;
    final double translationKp;
    final double translationKi;
    final double translationKd;
    final double headingKp;
    final double headingKi;
    final double headingKd;
    final double positionTolerance;
    final double headingToleranceDeg;
    final double period;
    final String pathFolder;
    final boolean telemetry;
    final String telemetryTable;

    private PointWeaverConfig(Builder b) {
        maxSpeed = b.maxSpeed;
        translationKp = b.translationKp;
        translationKi = b.translationKi;
        translationKd = b.translationKd;
        headingKp = b.headingKp;
        headingKi = b.headingKi;
        headingKd = b.headingKd;
        positionTolerance = b.positionTolerance;
        headingToleranceDeg = b.headingToleranceDeg;
        period = b.period;
        pathFolder = b.pathFolder;
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

    FollowerSettings toSettings() {
        return new FollowerSettings(maxSpeed, translationKp, translationKi, translationKd,
            positionTolerance, headingToleranceDeg, period);
    }

    public static final class Builder {
        private final double maxSpeed;
        private double translationKp = 1.0;
        private double translationKi = 0.0;
        private double translationKd = 0.0;
        private double headingKp = 2.0;
        private double headingKi = 0.0;
        private double headingKd = 0.0;
        private double positionTolerance = 0.05;
        private double headingToleranceDeg = 2.0;
        private double period = 0.02;
        private String pathFolder = "pointweaver";
        private boolean telemetry = true;
        private String telemetryTable = "PointWeaver";

        private Builder(double maxSpeed) {
            this.maxSpeed = maxSpeed;
        }

        /** Distance-to-point controller (output m/s per m of error). Default 1, 0, 0. */
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

        /** Folder inside the deploy directory that holds the path files. Default "pointweaver". */
        public Builder pathFolder(String folder) {
            pathFolder = folder;
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
