package pointweaver.core;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

/** Reads {@code robot.json}. Unknown fields are ignored. */
public final class RobotSettingsParser {
    private RobotSettingsParser() {}

    /**
     * @param file path to robot.json
     * @return the settings
     * @throws PathLoadException if the file is missing, unreadable or invalid
     */
    public static RobotSettings parse(Path file) {
        if (!Files.isRegularFile(file)) {
            throw new PathLoadException("robot.json not found at " + file.toAbsolutePath()
                + ". Create it in the PointWeaver app (Robot panel).");
        }
        try {
            return parse(Files.readString(file, StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new PathLoadException("Couldn't read robot.json: " + e.getMessage(), e);
        }
    }

    /**
     * @param json robot.json contents
     * @return the settings
     * @throws PathLoadException if the JSON is invalid or a field has the wrong type
     */
    public static RobotSettings parse(String json) {
        Object root;
        try {
            root = Json.parse(json);
        } catch (IllegalArgumentException e) {
            throw new PathLoadException("robot.json is not valid JSON: " + e.getMessage(), e);
        }
        if (!(root instanceof Map)) throw bad("the file must be an object");
        Map<?, ?> o = (Map<?, ?>) root;
        return new RobotSettings(
            positive(o, "maxSpeed"),
            positive(o, "maxAccel"),
            pid(o, "translationPID"),
            pid(o, "headingPID"),
            positive(o, "positionTolerance"),
            positive(o, "headingTolerance"),
            positive(o, "maxAngularRate"),
            positive(o, "robotLength"),
            positive(o, "robotWidth"),
            nonNegative(o, "responseLag"));
    }

    private static Double nonNegative(Map<?, ?> o, String key) {
        Object v = o.get(key);
        if (v == null) return null;
        if (!(v instanceof Double)) throw bad("'" + key + "' must be a number");
        if (!((Double) v >= 0)) throw bad("'" + key + "' can't be negative");
        return (Double) v;
    }

    private static Double positive(Map<?, ?> o, String key) {
        Object v = o.get(key);
        if (v == null) return null;
        if (!(v instanceof Double)) throw bad("'" + key + "' must be a number");
        if (!((Double) v > 0)) throw bad("'" + key + "' must be greater than 0");
        return (Double) v;
    }

    /** [kP], [kP, kI] or [kP, kI, kD]; missing terms are 0. */
    private static double[] pid(Map<?, ?> o, String key) {
        Object v = o.get(key);
        if (v == null) return null;
        if (!(v instanceof List) || ((List<?>) v).isEmpty() || ((List<?>) v).size() > 3) {
            throw bad("'" + key + "' must be a list like [kP, kI, kD]");
        }
        List<?> list = (List<?>) v;
        double[] out = new double[3];
        for (int i = 0; i < list.size(); i++) {
            if (!(list.get(i) instanceof Double)) throw bad("'" + key + "' must only contain numbers");
            out[i] = (Double) list.get(i);
        }
        return out;
    }

    private static PathLoadException bad(String msg) {
        return new PathLoadException("robot.json: " + msg);
    }
}
