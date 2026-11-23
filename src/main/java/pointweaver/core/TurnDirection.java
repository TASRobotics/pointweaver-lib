package pointweaver.core;

/**
 * Which way an interpolated turn goes round. Headings grow counterclockwise (WPILib's convention), so clockwise
 * means the heading goes down. In path files: {@code "turnDirection": "cw"}, {@code "ccw"} or {@code "auto"}.
 */
public enum TurnDirection {
    /** The shorter way round (the default). */
    AUTO,
    /** Clockwise: the heading goes down, even when the other way is shorter. */
    CW,
    /** Counterclockwise: the heading goes up, even when the other way is shorter. */
    CCW
}
