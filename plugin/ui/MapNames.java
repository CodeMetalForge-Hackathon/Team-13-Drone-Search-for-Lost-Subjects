package com.atakmap.android.plugintemplate.plugin.ui;

import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Names for map items the Search tab creates: the prefix and the next number ("RZ 3"). */
final class MapNames {

    private MapNames() {
    }

    /**
     * "prefix n" with n one above the highest number already used after the prefix (case-ignored),
     * so a new item never repeats a title. Titles with words after the prefix ("RZ village") have no
     * number and do not count; settings after the number ("Spot 2 r=200 s=5") do not matter.
     */
    static String next(String prefix, Iterable<String> titles) {
        Pattern p = Pattern.compile("^" + Pattern.quote(prefix.toLowerCase(Locale.ROOT))
                + "\\s+(\\d+)(\\s.*)?$");
        long max = 0;
        for (String t : titles) {
            if (t == null)
                continue;
            Matcher m = p.matcher(t.trim().toLowerCase(Locale.ROOT));
            if (m.matches()) {
                try {
                    max = Math.max(max, Long.parseLong(m.group(1)));
                } catch (NumberFormatException ignored) {
                    // a number too long to be a count
                }
            }
        }
        return prefix + " " + (max + 1);
    }
}
