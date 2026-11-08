package pointweaver.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Tiny JSON reader so the library doesn't depend on a JSON library
 * (WPILib 2026 ships Jackson, 2027 replaces it with Avaje Jsonb).
 *
 * <p>Produces {@code Map<String, Object>}, {@code List<Object>}, {@code String}, {@code Double},
 * {@code Boolean} or {@code null}.
 */
public final class Json {
    private final String src;
    private int pos;

    private Json(String src) {
        this.src = src;
    }

    /**
     * Parse a JSON document.
     *
     * @param text JSON text
     * @return the parsed value
     * @throws IllegalArgumentException with the position of the problem if the text isn't valid JSON
     */
    public static Object parse(String text) {
        Json p = new Json(text);
        p.skipWs();
        Object v = p.value();
        p.skipWs();
        if (p.pos != p.src.length()) throw p.error("unexpected trailing content");
        return v;
    }

    private Object value() {
        if (pos >= src.length()) throw error("unexpected end of input");
        char c = src.charAt(pos);
        switch (c) {
            case '{': return object();
            case '[': return array();
            case '"': return string();
            case 't': literal("true"); return Boolean.TRUE;
            case 'f': literal("false"); return Boolean.FALSE;
            case 'n': literal("null"); return null;
            default:
                if (c == '-' || (c >= '0' && c <= '9')) return number();
                throw error("unexpected character '" + c + "'");
        }
    }

    private Map<String, Object> object() {
        Map<String, Object> out = new LinkedHashMap<>();
        pos++; // {
        skipWs();
        if (peek() == '}') {
            pos++;
            return Collections.unmodifiableMap(out);
        }
        while (true) {
            skipWs();
            if (peek() != '"') throw error("expected a string key");
            String key = string();
            skipWs();
            expect(':');
            skipWs();
            out.put(key, value());
            skipWs();
            char c = next();
            if (c == '}') return Collections.unmodifiableMap(out);
            if (c != ',') throw error("expected ',' or '}'");
        }
    }

    private List<Object> array() {
        List<Object> out = new ArrayList<>();
        pos++; // [
        skipWs();
        if (peek() == ']') {
            pos++;
            return Collections.unmodifiableList(out);
        }
        while (true) {
            skipWs();
            out.add(value());
            skipWs();
            char c = next();
            if (c == ']') return Collections.unmodifiableList(out);
            if (c != ',') throw error("expected ',' or ']'");
        }
    }

    private String string() {
        pos++; // opening quote
        StringBuilder sb = new StringBuilder();
        while (true) {
            if (pos >= src.length()) throw error("unterminated string");
            char c = src.charAt(pos++);
            if (c == '"') return sb.toString();
            if (c != '\\') {
                sb.append(c);
                continue;
            }
            char e = next();
            switch (e) {
                case '"': sb.append('"'); break;
                case '\\': sb.append('\\'); break;
                case '/': sb.append('/'); break;
                case 'b': sb.append('\b'); break;
                case 'f': sb.append('\f'); break;
                case 'n': sb.append('\n'); break;
                case 'r': sb.append('\r'); break;
                case 't': sb.append('\t'); break;
                case 'u':
                    if (pos + 4 > src.length()) throw error("bad unicode escape");
                    try {
                        sb.append((char) Integer.parseInt(src.substring(pos, pos + 4), 16));
                    } catch (NumberFormatException ex) {
                        throw error("bad unicode escape");
                    }
                    pos += 4;
                    break;
                default: throw error("bad escape '\\" + e + "'");
            }
        }
    }

    private Double number() {
        int start = pos;
        if (peek() == '-') pos++;
        while (pos < src.length()) {
            char c = src.charAt(pos);
            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') pos++;
            else break;
        }
        try {
            return Double.valueOf(src.substring(start, pos));
        } catch (NumberFormatException ex) {
            pos = start;
            throw error("bad number");
        }
    }

    private void literal(String word) {
        if (!src.startsWith(word, pos)) throw error("expected '" + word + "'");
        pos += word.length();
    }

    private void skipWs() {
        while (pos < src.length() && Character.isWhitespace(src.charAt(pos))) pos++;
    }

    private char peek() {
        return pos < src.length() ? src.charAt(pos) : '\0';
    }

    private char next() {
        if (pos >= src.length()) throw error("unexpected end of input");
        return src.charAt(pos++);
    }

    private void expect(char c) {
        if (next() != c) {
            pos--;
            throw error("expected '" + c + "'");
        }
    }

    private IllegalArgumentException error(String msg) {
        int line = 1;
        int col = 1;
        for (int i = 0; i < Math.min(pos, src.length()); i++) {
            if (src.charAt(i) == '\n') {
                line++;
                col = 1;
            } else {
                col++;
            }
        }
        return new IllegalArgumentException(msg + " (line " + line + ", column " + col + ")");
    }
}
