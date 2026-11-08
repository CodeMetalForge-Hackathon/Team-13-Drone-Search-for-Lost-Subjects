package com.atakmap.android.plugintemplate.plugin.terrain;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Looks up terrain directly from DTED tiles laid out ATAK-style under one or more
 * root directories ({@code <root>/e012/n45.dt2}). Highest resolution level wins.
 */
public class BundledDtedSource implements ElevationSource {

    private static final String[] LEVELS = {"dt2", "dt1", "dt0"};
    private static final int MAX_OPEN = 8;

    private final List<File> roots;
    private final Map<String, DtedReader> open = new LinkedHashMap<String, DtedReader>(16, 0.75f,
            true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<String, DtedReader> eldest) {
            if (size() > MAX_OPEN) {
                try {
                    eldest.getValue().close();
                } catch (IOException ignored) {
                }
                return true;
            }
            return false;
        }
    };

    public BundledDtedSource(List<File> roots) {
        this.roots = new ArrayList<>(roots);
    }

    @Override
    public synchronized double getElevationMsl(double lat, double lon) {
        for (String ext : LEVELS) {
            for (File root : roots) {
                File f = findTile(root, lat, lon, ext);
                if (f == null)
                    continue;
                try {
                    DtedReader r = open.get(f.getPath());
                    if (r == null) {
                        r = new DtedReader(f);
                        open.put(f.getPath(), r);
                    }
                    double v = r.getElevation(lat, lon);
                    if (!Double.isNaN(v))
                        return v;
                } catch (IOException e) {
                    // unreadable/corrupt tile: try the next level or root
                }
            }
        }
        return Double.NaN;
    }

    /** "DTED2" / "DTED1" / "DTED0" for the best local tile covering the point, or "". */
    @Override
    public String sourceAt(double lat, double lon) {
        for (String ext : LEVELS)
            for (File root : roots)
                if (findTile(root, lat, lon, ext) != null)
                    return "DTED" + ext.charAt(2);
        return "";
    }

    public synchronized void close() {
        for (DtedReader r : open.values()) {
            try {
                r.close();
            } catch (IOException ignored) {
            }
        }
        open.clear();
    }

    /** ATAK / standard DTED relative path, e.g. "w077/n38.dt2". */
    public static String relativePath(double lat, double lon, String ext) {
        int latIdx = (int) Math.floor(lat);
        int lonIdx = (int) Math.floor(lon);
        return String.format(Locale.US, "%c%03d/%c%02d.%s",
                lonIdx < 0 ? 'w' : 'e', Math.abs(lonIdx),
                latIdx < 0 ? 's' : 'n', Math.abs(latIdx), ext);
    }

    private static File findTile(File root, double lat, double lon, String ext) {
        String rel = relativePath(lat, lon, ext);
        File f = new File(root, rel);
        if (f.isFile())
            return f;
        f = new File(root, rel.toUpperCase(Locale.US));
        return f.isFile() ? f : null;
    }
}
