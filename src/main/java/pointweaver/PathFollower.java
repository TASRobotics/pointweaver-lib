package pointweaver;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import pointweaver.core.FollowerCore;
import pointweaver.core.PathDefinition;

/**
 * Follows one path on the configured CTRE drivetrain. Shaped like an OpMode:
 * {@link #start()} once, {@link #update()} every loop, {@link #stop()} at the end.
 * Get one from {@link PointWeaver#follower(String)}.
 */
public final class PathFollower {
    private final FollowerCore core;
    private final SwerveDrivetrain<?, ?, ?> drivetrain;
    private final WeaverTelemetry telemetry;
    private final SwerveRequest.FieldCentricFacingAngle drive;
    private final SwerveRequest.RobotCentric idle = new SwerveRequest.RobotCentric()
        .withVelocityX(0).withVelocityY(0).withRotationalRate(0);
    private boolean started;

    PathFollower(PathDefinition path, SwerveDrivetrain<?, ?, ?> drivetrain, PointWeaverConfig config, WeaverTelemetry telemetry) {
        this.core = new FollowerCore(path, config.toSettings(), PointWeaver::fireEvent);
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;
        this.drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(config.headingKp, config.headingKi, config.headingKd);
    }

    /**
     * Begin the path from where the robot is now (resetting odometry first if the path asks for it).
     * Starts from the robot's current velocity, so chaining onto a moving robot doesn't jolt it.
     */
    public void start() {
        var state = drivetrain.getState();
        Pose2d pose = state.Pose;
        FollowerCore.Pose reset = core.resetPose(pose.getRotation().getDegrees());
        if (reset != null) {
            pose = new Pose2d(reset.x, reset.y, Rotation2d.fromDegrees(reset.headingDeg));
            drivetrain.resetPose(pose);
        }
        // measured speeds are robot-relative; the follower works in field coordinates
        ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        core.start(pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
            field.vxMetersPerSecond, field.vyMetersPerSecond, Timer.getFPGATimestamp());
        started = true;
        telemetry.pathStarted(core.path());
    }

    /** Drive for one loop. Starts the path first if {@link #start()} wasn't called. */
    public void update() {
        if (!started) start();
        Pose2d pose = drivetrain.getState().Pose;
        FollowerCore.Output out = core.update(pose.getX(), pose.getY(), pose.getRotation().getDegrees(), Timer.getFPGATimestamp());
        if (out.finished) {
            drivetrain.setControl(idle);
        } else {
            drivetrain.setControl(drive
                .withVelocityX(out.vx)
                .withVelocityY(out.vy)
                .withTargetDirection(Rotation2d.fromDegrees(out.headingDeg)));
        }
        telemetry.update(pose, core.path(), out);
        if (out.finished) telemetry.pathEnded();
    }

    /** True once the last point is reached (or the path couldn't be loaded). */
    public boolean isFinished() {
        return core.isFinished();
    }

    /** Stop driving now. Safe to call any time, including after the path finished. */
    public void stop() {
        core.cancel();
        started = false;
        drivetrain.setControl(idle);
        telemetry.pathEnded();
    }

    /** The path's name. */
    public String name() {
        return core.path().name;
    }
}
