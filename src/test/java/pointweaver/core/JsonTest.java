package pointweaver.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class JsonTest {
    @Test
    void parsesNestedValues() {
        Object v = Json.parse(" { \"a\": [1, -2.5, 3e2, true, false, null], \"b\": {\"c\": \"x\\ny\\u0041\"}, \"e\": {} } ");
        Map<?, ?> m = (Map<?, ?>) v;
        assertEquals(List.of(1.0, -2.5, 300.0, true, false), ((List<?>) m.get("a")).subList(0, 5));
        assertNull(((List<?>) m.get("a")).get(5));
        assertEquals("x\nyA", ((Map<?, ?>) m.get("b")).get("c"));
        assertTrue(((Map<?, ?>) m.get("e")).isEmpty());
    }

    @Test
    void reportsWhereItBroke() {
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> Json.parse("{\n  \"a\": 1,\n  \"b\" 2\n}"));
        assertTrue(e.getMessage().contains("line 3"), e.getMessage());
        assertThrows(IllegalArgumentException.class, () -> Json.parse("[1, 2"));
        assertThrows(IllegalArgumentException.class, () -> Json.parse("{} extra"));
        assertThrows(IllegalArgumentException.class, () -> Json.parse("\"unterminated"));
    }
}
