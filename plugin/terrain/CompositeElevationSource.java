package com.atakmap.android.plugintemplate.plugin.terrain;

import java.util.Arrays;
import java.util.List;

/** Returns the first non-NaN answer from an ordered list of sources. */
public class CompositeElevationSource implements ElevationSource {

    private final List<ElevationSource> sources;

    public CompositeElevationSource(ElevationSource... sources) {
        this.sources = Arrays.asList(sources);
    }

    @Override
    public double getElevationMsl(double lat, double lon) {
        for (ElevationSource s : sources) {
            double v = s.getElevationMsl(lat, lon);
            if (!Double.isNaN(v))
                return v;
        }
        return Double.NaN;
    }

    @Override
    public String sourceAt(double lat, double lon) {
        for (ElevationSource s : sources)
            if (!Double.isNaN(s.getElevationMsl(lat, lon)))
                return s.sourceAt(lat, lon);
        return "";
    }
}
