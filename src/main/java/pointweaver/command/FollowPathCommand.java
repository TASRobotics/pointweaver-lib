package pointweaver.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import pointweaver.PathFollower;
import pointweaver.PointWeaver;

/** Command-based wrapper around a {@link PathFollower}. Stops the drivetrain when it ends or is interrupted. */
public final class FollowPathCommand extends Command {
    private final PathFollower follower;

    public FollowPathCommand(PathFollower follower) {
        this.follower = follower;
        if (PointWeaver.drivetrain() instanceof Subsystem) addRequirements((Subsystem) PointWeaver.drivetrain());
        setName("FollowPath(" + follower.name() + ")");
    }

    @Override
    public void initialize() {
        follower.start();
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        follower.stop();
    }
}
