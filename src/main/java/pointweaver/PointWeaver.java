package pointweaver;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Filesystem;

import java.nio.file.Path;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import pointweaver.core.PathDefinition;
import pointweaver.core.PathLoadException;
import pointweaver.core.PathParser;

/**
 * Entry point. Configure once, register events, then follow paths:
 *
 * <pre>{@code
 * // robot constructor (settings come from deploy/pointweaver/robot.json)
 * PointWeaver.configure(drivetrain);
 * PointWeaver.registerEvent("intake", () -> intake.start());
 *
 * // auto
 * PathFollower follower = PointWeaver.follower("testpath");
 * follower.start();          // once
 * follower.update();         // every loop until follower.isFinished()
 * follower.stop();           // when the auto ends
 * }</pre>
 *
 * <p>Command-based robots can use {@code pointweaver.command.PointWeaverCommands} (Commands v2) or
 * {@code pointweaver.command3.PointWeaverCommands} (Commands v3) instead.
 * Paths are read from {@code src/main/deploy/pointweaver/paths/<name>.json}, the folder the PointWeaver app saves to.
 */
public final class PointWeaver {
    private static SwerveDrivetrain<?, ?, ?> drivetrain;
    private static PointWeaverConfig config;
    private static WeaverTelemetry telemetry;
    private static final Map<String, Runnable> events = new ConcurrentHashMap<>();
    private static final Set<String> warnedEvents = ConcurrentHashMap.newKeySet();
    private static final Map<String, PathDefinition> cache = new ConcurrentHashMap<>();

    private PointWeaver() {}

    /**
     * Tell PointWeaver which drivetrain to drive, with settings from {@code deploy/pointweaver/robot.json}.
     * Call once, in the robot constructor.
     *
     * @param drivetrain any CTRE Phoenix 6 swerve drivetrain (e.g. the Tuner X generated one)
     * @throws pointweaver.core.PathLoadException if robot.json is missing or invalid
     */
    public static void configure(SwerveDrivetrain<?, ?, ?> drivetrain) {
        configure(drivetrain, PointWeaverConfig.fromDeploy().build());
    }

    /**
     * Tell PointWeaver which drivetrain to drive, with settings from code. Call once, in the robot constructor.
     *
     * @param drivetrain any CTRE Phoenix 6 swerve drivetrain (e.g. the Tuner X generated one)
     * @param config robot limits and tuning
     */
    public static synchronized void configure(SwerveDrivetrain<?, ?, ?> drivetrain, PointWeaverConfig config) {
        PointWeaver.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain");
        PointWeaver.config = Objects.requireNonNull(config, "config");
        PointWeaver.telemetry = new WeaverTelemetry(config.telemetry, config.telemetryTable);
        cache.clear();
    }

    public static boolean isConfigured() {
        return drivetrain != null;
    }

    /**
     * Say what an event (named in the PointWeaver app) does. It runs on the robot loop, so keep it quick:
     * start something, don't wait for it.
     *
     * @param name event name, exactly as in the app (case sensitive)
     * @param action what to do when the event fires
     */
    public static void registerEvent(String name, Runnable action) {
        events.put(Objects.requireNonNull(name, "name"), Objects.requireNonNull(action, "action"));
    }

    /**
     * Load a path from the deploy folder (cached after the first load).
     *
     * @param name file name without .json (case sensitive)
     * @return the parsed path
     * @throws PathLoadException if the file is missing or invalid
     */
    public static PathDefinition loadPath(String name) {
        Objects.requireNonNull(name, "name");
        String folder = config != null ? config.deployFolder : PointWeaverConfig.DEFAULT_DEPLOY_FOLDER;
        return cache.computeIfAbsent(name.trim(), n -> {
            Path file = Filesystem.getDeployDirectory().toPath().resolve(folder).resolve("paths").resolve(n + ".json");
            return PathParser.parse(file);
        });
    }

    /**
     * A follower for a path file. Never throws: if the file can't be loaded, the error goes to the
     * Driver Station and you get a follower that finishes immediately and keeps the robot still.
     *
     * @param name file name without .json (case sensitive)
     */
    public static PathFollower follower(String name) {
        PathDefinition path;
        try {
            path = loadPath(name);
        } catch (PathLoadException e) {
            DriverStationErrors.reportError("PointWeaver: " + e.getMessage(), false);
            path = PathDefinition.empty(name);
        }
        return follower(path);
    }

    /** A follower for an already loaded path. */
    public static PathFollower follower(PathDefinition path) {
        requireConfigured();
        return new PathFollower(path, drivetrain, config, telemetry);
    }

    /** Forget loaded paths so the next load reads the files again. */
    public static void clearCache() {
        cache.clear();
    }

    /** The configured drivetrain (null before {@link #configure}). */
    public static SwerveDrivetrain<?, ?, ?> drivetrain() {
        return drivetrain;
    }

    static void fireEvent(String name) {
        Runnable action = events.get(name);
        if (telemetry != null) telemetry.event(name);
        if (action == null) {
            if (warnedEvents.add(name)) {
                DriverStationErrors.reportWarning("PointWeaver: no event registered for '" + name + "'", false);
            }
            return;
        }
        action.run();
    }

    private static void requireConfigured() {
        if (!isConfigured()) {
            throw new IllegalStateException("Call PointWeaver.configure(drivetrain, config) before following paths");
        }
    }
}
