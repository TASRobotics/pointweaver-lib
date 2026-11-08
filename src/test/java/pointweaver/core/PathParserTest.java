package pointweaver.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Test;

class PathParserTest {
    static Path fixture(String name) throws Exception {
        return Paths.get(PathParserTest.class.getResource("/paths/" + name + ".json").toURI());
    }

    @Test
    void readsTheAppFormat() throws Exception {
        PathDefinition def = PathParser.parse(fixture("testpath"));
        assertEquals("testpath", def.name);
        assertEquals(8, def.points.size());
        assertEquals(3.0, def.params.maxSpeed);
        assertEquals(true, def.params.interpolate);
        assertNull(def.points.get(0).theta);
        assertEquals(-135.0, def.points.get(1).theta);
        assertEquals(2.0, def.points.get(1).params.minSpeed);
        assertNull(def.points.get(7).params.minSpeed);
        assertTrue(def.points.get(0).events.isEmpty());
        assertFalse(def.resetPose);
    }

    @Test
    void readsEventsResetPoseNullsAndIgnoresUnknownFields() {
        PathDefinition def = PathParser.parse("p", "{"
            + "\"resetPose\": true, \"params\": null, \"futureField\": [1,2],"
            + "\"points\": [{\"x\": 1, \"y\": 2, \"theta\": null, \"params\": {\"maxSpeed\": null, \"timeout\": 1.5},"
            + "  \"events\": [{\"name\": \"intake\", \"fraction\": 0.25}, {\"name\": \"no_fraction\"}]}]}");
        assertTrue(def.resetPose);
        assertNull(def.params.maxSpeed);
        PathPoint p = def.points.get(0);
        assertNull(p.theta);
        assertEquals(1.5, p.params.timeout);
        assertEquals("intake", p.events.get(0).name);
        assertEquals(0.25, p.events.get(0).fraction);
        assertEquals(0.0, p.events.get(1).fraction);
    }

    @Test
    void explainsBadFiles() {
        PathLoadException e = assertThrows(PathLoadException.class, () -> PathParser.parse("bad", "{\"points\": [{\"x\": 1}]}"));
        assertTrue(e.getMessage().contains("'bad'") && e.getMessage().contains("point 0"), e.getMessage());
        e = assertThrows(PathLoadException.class, () -> PathParser.parse("bad", "{\"points\": [{\"x\": \"1\", \"y\": 2}]}"));
        assertTrue(e.getMessage().contains("must be a number"), e.getMessage());
        e = assertThrows(PathLoadException.class, () -> PathParser.parse("bad", "{nope"));
        assertTrue(e.getMessage().contains("not valid JSON"), e.getMessage());
        e = assertThrows(PathLoadException.class, () -> PathParser.parse(Paths.get("does/not/exist.json")));
        assertTrue(e.getMessage().contains("not found"), e.getMessage());
    }

    @Test
    void missingPointsMeansEmptyPath() {
        assertTrue(PathParser.parse("e", "{}").points.isEmpty());
    }
}
