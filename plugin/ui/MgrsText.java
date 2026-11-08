package com.atakmap.android.plugintemplate.plugin.ui;

import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Typed MGRS grids, made exact before ATAK converts them. A grid has 6, 8 or 10 digits: 100 m,
 * 10 m or 1 m. The digits split into equal halves, easting then northing ("123 456" is easting 123
 * and northing 456 in hundreds of metres), and each half is padded to metres: the grid names the
 * south-west corner of its square ("11S PU 123 456" becomes "11S PU 12300 45600"). Typed apart,
 * the halves must have the same length. UPS (polar) grids are not accepted.
 */
final class MgrsText {

    /**
     * Any run of spaces (Unicode ones included: no-break, thin) and invisible format characters.
     * ATAK formats MGRS with a left-to-right mark (U+200E) after the zone and the square, so a grid
     * copied from ATAK carries them.
     */
    static final String SPACES = "[\\s\\p{Z}\\p{Cf}]+";
    static final int MIN_DIGITS = 6;
    static final int MAX_DIGITS = 10;

    /** Zone, latitude band (C-X without I, O), 100 km column (A-Z without I, O), row (A-V without I, O), digits. */
    private static final Pattern GRID = Pattern.compile(
            "^(\\d{1,2}) ?([C-HJ-NP-X]) ?([A-HJ-NP-Z]) ?([A-HJ-NP-V]) ?([0-9 ]*)$");

    private MgrsText() {
    }

    /** The grid to the metre, in ATAK's spacing, or null when the text is not a 6 to 10 digit grid. */
    static String canonical(String text) {
        if (text == null)
            return null;
        String t = text.replaceAll("\\p{Cf}", "").replaceAll(SPACES, " ").trim()
                .toUpperCase(Locale.ROOT);
        Matcher m = GRID.matcher(t);
        if (!m.matches())
            return null;
        int zone = Integer.parseInt(m.group(1));
        if (zone < 1 || zone > 60)
            return null;
        String digits = m.group(5).trim();
        String easting, northing;
        String[] parts = digits.isEmpty() ? new String[0] : digits.split(" ");
        if (parts.length == 1) {
            String d = parts[0];
            if (d.length() % 2 != 0)
                return null;
            easting = d.substring(0, d.length() / 2);
            northing = d.substring(d.length() / 2);
        } else if (parts.length == 2) {
            easting = parts[0];
            northing = parts[1];
            if (easting.length() != northing.length())
                return null;
        } else {
            return null;
        }
        int total = easting.length() + northing.length();
        if (total < MIN_DIGITS || total > MAX_DIGITS)
            return null;
        return zone + m.group(2) + " " + m.group(3) + m.group(4) + " " + metres(easting) + " "
                + metres(northing);
    }

    /** A half padded to five digits: the south-west corner of its square. */
    private static String metres(String half) {
        StringBuilder sb = new StringBuilder(half);
        while (sb.length() < 5)
            sb.append('0');
        return sb.toString();
    }
}
