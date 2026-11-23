package pointweaver.command3;

import java.util.Objects;
import java.util.Set;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

import pointweaver.PathFollower;

/**
 * Commands v3 wrapper around a {@link PathFollower}: starts it, updates it every loop until the path finishes, and
 * stops the drivetrain when it ends, is canceled or is interrupted. Get one from {@link PointWeaverCommands#followPath}.
 */
public final class FollowPathCommand implements Command {
    private final PathFollower follower;
    private final Set<Mechanism> requirements;

    /**
     * @param follower the path to follow
     * @param drivetrain the mechanism that owns the swerve, so nothing else (like the joystick drive command) drives it
     *     at the same time
     */
    public FollowPathCommand(PathFollower follower, Mechanism drivetrain) {
        this.follower = Objects.requireNonNull(follower, "follower");
        this.requirements = Set.of(Objects.requireNonNull(drivetrain, "drivetrain"));
    }

    @Override
    public void run(Coroutine coroutine) {
        follower.start();
        while (!follower.isFinished()) {
            follower.update();
            coroutine.yield();
        }
    }

    /** Runs however the command ends: finished, canceled or interrupted. */
    @Override
    public void onExit() {
        follower.stop();
    }

    @Override
    public String name() {
        return "FollowPath(" + follower.name() + ")";
    }

    @Override
    public Set<Mechanism> requirements() {
        return requirements;
    }
}
