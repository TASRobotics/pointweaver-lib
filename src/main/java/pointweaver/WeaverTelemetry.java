package pointweaver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import pointweaver.core.FollowerCore;
import pointweaver.core.PathDefinition;
import pointweaver.core.PathPoint;

/**
 * Follower state on NetworkTables (default table "PointWeaver"). Pose topics are Pose2d structs, so
 * AdvantageScope and Elastic can draw the robot, the target and the whole path on a field.
 * The only place the library talks to NetworkTables.
 */
final class WeaverTelemetry {
    private final boolean enabled;
    private StructPublisher<Pose2d> pose;
    private StructPublisher<Pose2d> target;
    private StructArrayPublisher<Pose2d> path;
    private StringPublisher activePath;
    private StringPublisher lastEvent;
    private BooleanPublisher running;
    private IntegerPublisher targetIndex;
    private DoublePublisher distance;
    private DoublePublisher desiredHeading;
    private DoublePublisher vx;
    private DoublePublisher vy;

    WeaverTelemetry(boolean enabled, String tableName) {
        this.enabled = enabled;
        if (!enabled) return;
        NetworkTable t = NetworkTableInstance.getDefault().getTable(tableName);
        pose = t.getStructTopic("pose", Pose2d.struct).publish();
        target = t.getStructTopic("target", Pose2d.struct).publish();
        path = t.getStructArrayTopic("path", Pose2d.struct).publish();
        activePath = t.getStringTopic("activePath").publish();
        lastEvent = t.getStringTopic("lastEvent").publish();
        running = t.getBooleanTopic("running").publish();
        targetIndex = t.getIntegerTopic("targetIndex").publish();
        distance = t.getDoubleTopic("distance").publish();
        desiredHeading = t.getDoubleTopic("desiredHeadingDeg").publish();
        vx = t.getDoubleTopic("vx").publish();
        vy = t.getDoubleTopic("vy").publish();
    }

    void pathStarted(PathDefinition def) {
        if (!enabled) return;
        Pose2d[] poses = new Pose2d[def.points.size()];
        for (int i = 0; i < poses.length; i++) poses[i] = toPose(def.points.get(i));
        path.set(poses);
        activePath.set(def.name);
        running.set(true);
    }

    void update(Pose2d robot, PathDefinition def, FollowerCore.Output out) {
        if (!enabled) return;
        pose.set(robot);
        if (!def.points.isEmpty()) target.set(toPose(def.points.get(out.targetIndex)));
        targetIndex.set(out.targetIndex);
        distance.set(out.distance);
        desiredHeading.set(out.headingDeg);
        vx.set(out.vx);
        vy.set(out.vy);
    }

    void pathEnded() {
        if (!enabled) return;
        running.set(false);
        vx.set(0);
        vy.set(0);
    }

    void event(String name) {
        if (enabled) lastEvent.set(name);
    }

    private static Pose2d toPose(PathPoint p) {
        return new Pose2d(p.x, p.y, Rotation2d.fromDegrees(p.theta != null ? p.theta : 0));
    }
}
