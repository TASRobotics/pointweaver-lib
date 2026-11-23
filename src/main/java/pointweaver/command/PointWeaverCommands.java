package pointweaver.command;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

import pointweaver.PointWeaver;
import pointweaver.core.PathDefinition;

/**
 * Helpers for command-based (Commands v2) robots. Everything else in PointWeaver works without the
 * command framework; this is the only package that uses it. Commands v3 robots use
 * {@link pointweaver.command3.PointWeaverCommands}.
 */
public final class PointWeaverCommands {
    private PointWeaverCommands() {}

    /** A command that follows a path file (see {@link PointWeaver#follower(String)}). */
    public static Command followPath(String name) {
        return new FollowPathCommand(PointWeaver.follower(name));
    }

    /** A command that follows an already loaded path. */
    public static Command followPath(PathDefinition path) {
        return new FollowPathCommand(PointWeaver.follower(path));
    }

    /**
     * Schedule a command when an event fires. It runs alongside the path.
     * !!! Don't make it require the drivetrain, or it will interrupt the path.
     */
    public static void registerEvent(String name, Command command) {
        PointWeaver.registerEvent(name, () -> CommandScheduler.getInstance().schedule(command));
    }
}
