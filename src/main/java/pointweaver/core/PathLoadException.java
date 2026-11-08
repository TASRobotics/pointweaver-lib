package pointweaver.core;

/** A path file couldn't be found, read, or understood. The message says which file and why. */
public class PathLoadException extends RuntimeException {
    private static final long serialVersionUID = 1L;

    public PathLoadException(String message) {
        super(message);
    }

    public PathLoadException(String message, Throwable cause) {
        super(message, cause);
    }
}
