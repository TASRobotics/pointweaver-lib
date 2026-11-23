package pointweaver.core;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Reads the PointWeaver path JSON (the format the PointWeaver app saves). Unknown fields are ignored,
 * so newer files still load on older library versions.
 */
public final class PathParser {
    private PathParser() {}

    /**
     * Read and parse a path file.
     *
     * @param file the .json file
     * @return the path, named after the file (without .json)
     * @throws PathLoadException if the file is missing, unreadable or invalid
     */
    public static PathDefinition parse(Path file) {
        String fileName = file.getFileName().toString();
        String name = fileName.endsWith(".json") ? fileName.substring(0, fileName.length() - 5) : fileName;
        if (!Files.isRegularFile(file)) {
            throw new PathLoadException("Path '" + name + "' not found at " + file.toAbsolutePath());
        }
        try {
            return parse(name, Files.readString(file, StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new PathLoadException("Couldn't read path '" + name + "': " + e.getMessage(), e);
        }
    }

    /**
     * Parse path JSON text.
     *
     * @param name name to report in errors and store on the result
     * @param json the JSON text
     * @return the path
     * @throws PathLoadException if the JSON is invalid or a field has the wrong type
     */
    public static PathDefinition parse(String name, String json) {
        Object root;
        try {
            root = Json.parse(json);
        } catch (IllegalArgumentException e) {
            throw new PathLoadException("Path '" + name + "' is not valid JSON: " + e.getMessage(), e);
        }
        Map<String, Object> obj = asObject(root, name, "the file");

        List<PathPoint> points = new ArrayList<>();
        Object rawPoints = obj.get("points");
        if (rawPoints != null) {
            if (!(rawPoints instanceof List)) throw bad(name, "'points' must be a list");
            List<?> list = (List<?>) rawPoints;
            for (int i = 0; i < list.size(); i++) points.add(point(list.get(i), name, i));
        }

        return new PathDefinition(
            name,
            points,
            params(obj.get("params"), name, "path params"),
            Boolean.TRUE.equals(bool(obj.get("resetPose"), name, "resetPose")));
    }

    private static PathPoint point(Object raw, String name, int i) {
        String where = "point " + i;
        Map<String, Object> p = asObject(raw, name, where);
        Double x = num(p.get("x"), name, where + " x");
        Double y = num(p.get("y"), name, where + " y");
        if (x == null || y == null) throw bad(name, where + " needs both x and y");

        List<PathEvent> events = new ArrayList<>();
        Object rawEvents = p.get("events");
        if (rawEvents != null) {
            if (!(rawEvents instanceof List)) throw bad(name, where + " events must be a list");
            List<?> list = (List<?>) rawEvents;
            for (int e = 0; e < list.size(); e++) {
                String ew = where + " event " + e;
                Map<String, Object> ev = asObject(list.get(e), name, ew);
                Object evName = ev.get("name");
                if (!(evName instanceof String)) throw bad(name, ew + " needs a name");
                Double fraction = num(ev.get("fraction"), name, ew + " fraction");
                events.add(new PathEvent((String) evName, fraction != null ? fraction : 0.0));
            }
        }

        return new PathPoint(x, y, num(p.get("theta"), name, where + " theta"), params(p.get("params"), name, where + " params"), events);
    }

    private static PathParams params(Object raw, String name, String where) {
        if (raw == null) return PathParams.EMPTY;
        Map<String, Object> p = asObject(raw, name, where);
        return new PathParams(
            num(p.get("minSpeed"), name, where + " minSpeed"),
            num(p.get("maxSpeed"), name, where + " maxSpeed"),
            num(p.get("maxAccel"), name, where + " maxAccel"),
            num(p.get("earlyExitRange"), name, where + " earlyExitRange"),
            num(p.get("timeout"), name, where + " timeout"),
            bool(p.get("interpolate"), name, where + " interpolate"),
            turn(p.get("turnDirection"), name, where + " turnDirection"));
    }

    private static TurnDirection turn(Object raw, String name, String where) {
        if (raw == null) return null;
        if (raw instanceof String s) {
            switch (s.toLowerCase(Locale.ROOT)) {
                case "cw": return TurnDirection.CW;
                case "ccw": return TurnDirection.CCW;
                case "auto": return TurnDirection.AUTO;
                default: break;
            }
        }
        throw bad(name, where + " must be \"cw\", \"ccw\" or \"auto\"");
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> asObject(Object raw, String name, String where) {
        if (!(raw instanceof Map)) throw bad(name, where + " must be an object");
        return (Map<String, Object>) raw;
    }

    private static Double num(Object raw, String name, String where) {
        if (raw == null) return null;
        if (!(raw instanceof Double)) throw bad(name, where + " must be a number");
        return (Double) raw;
    }

    private static Boolean bool(Object raw, String name, String where) {
        if (raw == null) return null;
        if (!(raw instanceof Boolean)) throw bad(name, where + " must be true or false");
        return (Boolean) raw;
    }

    private static PathLoadException bad(String name, String msg) {
        return new PathLoadException("Path '" + name + "': " + msg);
    }
}
