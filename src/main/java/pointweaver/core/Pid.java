package pointweaver.core;

/**
 * Minimal PID with a fixed loop period. Matches WPILib's PIDController math (including the default
 * integrator clamp of +/-1 output) so behavior is the same as before, without depending on WPILib.
 */
final class Pid {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double period;
    private double prevError;
    private double totalError;

    Pid(double kP, double kI, double kD, double period) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.period = period;
    }

    double calculate(double measurement, double setpoint) {
        double error = setpoint - measurement;
        if (kI != 0) {
            totalError = PathMath.clamp(totalError + error * period, -1.0 / kI, 1.0 / kI);
        }
        double derivative = (error - prevError) / period;
        prevError = error;
        return kP * error + kI * totalError + kD * derivative;
    }

    void reset() {
        prevError = 0;
        totalError = 0;
    }
}
