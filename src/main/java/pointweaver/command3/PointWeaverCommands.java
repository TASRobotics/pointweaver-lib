package pointweaver.command3;

import java.util.Objects;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationErrors;

import pointweaver.PointWeaver;
import pointweaver.core.PathDefinition;

/**
 * Helpers for Commands v3 robots (the Commands v3 vendordep). Commands v2 robots use
 * {@link pointweaver.command.PointWeaverCommands} instead; everything else in PointWeaver works without either.
 *
 * <pre>{@code
 * // robot constructor, after PointWeaver.configure(drivetrain)
 * PointWeaverCommands.registerEvent("intake", intake.runIntake());
 *
 * // auto
 * Scheduler.getDefault().schedule(PointWeaverCommands.followPath("testpath"));
 * }</pre>
 */
public final class PointWeaverCommands {
    /** pathMechanisms: what the command running the path required (empty if a PathFollower fired it outside commands) */
    private record Fired(String name, Command command, Set<Mechanism> pathMechanisms) {}

    /** Event commands that fired, waiting for the scheduler's next loop. */
    private static final Queue<Fired> fired = new ConcurrentLinkedQueue<>();
    /** The scheduler that schedules them (null until the first event is registered). */
    private static Scheduler scheduler;
    private static volatile long lastLoopNanos;
    private static volatile boolean warnedNotScheduling;

    private PointWeaverCommands() {}

    /**
     * A command that follows a path file (see {@link PointWeaver#follower(String)}), requiring the drivetrain.
     *
     * @throws IllegalStateException if the drivetrain passed to {@link PointWeaver#configure} isn't a
     *     {@link Mechanism}: use {@link #followPath(String, Mechanism)} with the mechanism that owns it
     */
    public static Command followPath(String name) {
        return followPath(name, drivetrainMechanism());
    }

    /**
     * A command that follows a path file (see {@link PointWeaver#follower(String)}).
     *
     * @param drivetrain the mechanism that owns the swerve; the command requires it
     */
    public static Command followPath(String name, Mechanism drivetrain) {
        return new FollowPathCommand(PointWeaver.follower(name), drivetrain);
    }

    /** A command that follows an already loaded path, requiring the drivetrain (see {@link #followPath(String)}). */
    public static Command followPath(PathDefinition path) {
        return followPath(path, drivetrainMechanism());
    }

    /** A command that follows an already loaded path, requiring {@code drivetrain}. */
    public static Command followPath(PathDefinition path, Mechanism drivetrain) {
        return new FollowPathCommand(PointWeaver.follower(path), drivetrain);
    }

    /**
     * Schedule a command when an event fires. Call this in the robot constructor, where trigger bindings go.
     *
     * <p>The command starts on the scheduler's next loop (20 ms later) as its own command, not as a child of the
     * path, so it keeps running after the path ends, until it finishes or the auto ends. If it needs a mechanism the
     * path is still using (the drivetrain, while the path or the auto around it runs), it isn't started, so it can't
     * cancel the auto; an error goes to the Driver Station instead. A last-point event fires as the path finishes,
     * so it may drive if nothing else is holding the drivetrain.
     *
     * @param name event name, exactly as in the app (case sensitive)
     * @param command what to run
     * @throws IllegalStateException if called from inside a running command
     */
    public static synchronized void registerEvent(String name, Command command) {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(command, "command");
        Scheduler s = Scheduler.getDefault();
        // Anything added to the scheduler from inside a command only lives as long as that command
        if (s.currentCommand() != null) {
            throw new IllegalStateException("Register PointWeaver event '" + name + "' in the robot constructor, not from a command");
        }
        if (scheduler != s) {
            scheduler = s;
            // runs at the start of every scheduler loop, outside any command
            s.addPeriodic(PointWeaverCommands::scheduleFired);
        }
        PointWeaver.registerEvent(name, () -> fire(name, command));
    }

    private static void fire(String name, Command command) {
        Command path = scheduler.currentCommand(); // the command following the path, if it runs in one
        fired.add(new Fired(name, command, path != null ? path.requirements() : Set.of()));
        if (!warnedNotScheduling && System.nanoTime() - lastLoopNanos > TimeUnit.MILLISECONDS.toNanos(100)) {
            warnedNotScheduling = true;
            DriverStationErrors.reportWarning("PointWeaver: event '" + name + "' fired, but the scheduler isn't picking up"
                + " event commands. Run Scheduler.getDefault().run() every loop and register events in the robot constructor.", false);
        }
    }

    private static void scheduleFired() {
        lastLoopNanos = System.nanoTime();
        for (Fired f; (f = fired.poll()) != null; ) {
            Mechanism taken = stillInUse(f);
            if (taken != null) {
                DriverStationErrors.reportError("PointWeaver: event '" + f.name + "' not run: " + f.command.name()
                    + " needs " + taken.getName() + ", which the path is still using", false);
                continue;
            }
            Scheduler.ScheduleResult result = scheduler.schedule(f.command);
            if (!result.successful()) {
                DriverStationErrors.reportWarning("PointWeaver: event '" + f.name + "' not run: "
                    + result.getClass().getSimpleName(), false);
            }
        }
    }

    /**
     * A mechanism the event's command needs that the path's command (or a group running it) still holds, or null.
     * Default commands don't count: an event taking over from those is normal.
     */
    private static Mechanism stillInUse(Fired f) {
        for (Mechanism m : f.command.requirements()) {
            if (!f.pathMechanisms.contains(m)) continue;
            for (Command holder : scheduler.getRunningCommandsFor(m)) {
                if (holder != f.command && holder != scheduler.getDefaultCommandFor(m)) return m;
            }
        }
        return null;
    }

    private static Mechanism drivetrainMechanism() {
        if (PointWeaver.drivetrain() instanceof Mechanism m) return m;
        throw new IllegalStateException(PointWeaver.isConfigured()
            ? "The PointWeaver drivetrain isn't a Commands v3 Mechanism: pass the mechanism that owns it, followPath(name, swerve)"
            : "Call PointWeaver.configure(drivetrain) before making path commands");
    }
}
