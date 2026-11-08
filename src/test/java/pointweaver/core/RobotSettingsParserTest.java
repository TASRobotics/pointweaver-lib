package pointweaver.core;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Paths;
import org.junit.jupiter.api.Test;

class RobotSettingsParserTest {
    @Test
    void readsAFullFile() {
        RobotSettings s = RobotSettingsParser.parse("{"
            + "\"maxSpeed\": 6.48, \"maxAccel\": 3, \"translationPID\": [5, 0, 0.1], \"headingPID\": [2],"
            + "\"positionTolerance\": 0.05, \"headingTolerance\": 2, \"maxAngularRate\": 918,"
            + "\"robotLength\": 0.81, \"robotWidth\": 0.83, \"somethingNew\": true}");
        assertEquals(6.48, s.maxSpeed);
        assertEquals(3.0, s.maxAccel);
        assertArrayEquals(new double[] {5, 0, 0.1}, s.translationPID);
        assertArrayEquals(new double[] {2, 0, 0}, s.headingPID, "missing I and D are 0");
        assertEquals(918.0, s.maxAngularRate);
        assertEquals(0.83, s.robotWidth);
    }

    @Test
    void everythingIsOptional() {
        RobotSettings s = RobotSettingsParser.parse("{\"maxSpeed\": 4.5}");
        assertEquals(4.5, s.maxSpeed);
        assertNull(s.maxAccel);
        assertNull(s.translationPID);
    }

    @Test
    void explainsBadValues() {
        assertMessage("{\"maxSpeed\": \"fast\"}", "'maxSpeed' must be a number");
        assertMessage("{\"maxAccel\": 0}", "'maxAccel' must be greater than 0");
        assertMessage("{\"headingPID\": [1, 2, 3, 4]}", "'headingPID' must be a list");
        assertMessage("{\"headingPID\": [\"a\"]}", "'headingPID' must only contain numbers");
        assertMessage("[1]", "must be an object");
        PathLoadException e = assertThrows(PathLoadException.class, () -> RobotSettingsParser.parse(Paths.get("nope/robot.json")));
        assertTrue(e.getMessage().contains("PointWeaver app"), e.getMessage());
    }

    private static void assertMessage(String json, String expected) {
        PathLoadException e = assertThrows(PathLoadException.class, () -> RobotSettingsParser.parse(json));
        assertTrue(e.getMessage().contains(expected), e.getMessage());
    }
}
