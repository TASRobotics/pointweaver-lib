package pointweaver.core;

/** A PointWeaver file (a path or robot.json) couldn't be found, read, or understood. The message says which and why. */
public class PathLoadException extends RuntimeException {
    private static final long serialVersionUID = 1L;

    public PathLoadException(String message) {
        super(message);
    }

    public PathLoadException(String message, Throwable cause) {
        super(message, cause);
    }
}
