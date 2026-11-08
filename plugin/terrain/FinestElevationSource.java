package com.atakmap.android.plugintemplate.plugin.terrain;

/**
 * Terrain from the finest data available at each point: local DTED2 or DTED1 first, then ATAK's own
 * elevation (DTED, SRTM or other data the operator loaded), then DTED0. Outside California the DTED
 * server only offers DTED0 (about 900 m between posts), which would flatten ridges and valleys in
 * the probability map and in the mission altitudes when better data is loaded in ATAK.
 */
public class FinestElevationSource implements ElevationSource {

    private final ElevationSource dted;
    private final ElevationSource fallback;

    /** @param dted local DTED tiles (reports DTED2 / DTED1 / DTED0 from sourceAt) */
    public FinestElevationSource(ElevationSource dted, ElevationSource fallback) {
        this.dted = dted;
        this.fallback = fallback;
    }

    private boolean fineDted(double lat, double lon) {
        String level = dted.sourceAt(lat, lon);
        return "DTED2".equals(level) || "DTED1".equals(level);
    }

    @Override
    public double getElevationMsl(double lat, double lon) {
        if (fineDted(lat, lon)) {
            double v = dted.getElevationMsl(lat, lon);
            if (!Double.isNaN(v))
                return v;
        }
        double a = fallback.getElevationMsl(lat, lon);
        if (!Double.isNaN(a))
            return a;
        return dted.getElevationMsl(lat, lon);
    }

    @Override
    public String sourceAt(double lat, double lon) {
        if (fineDted(lat, lon) && !Double.isNaN(dted.getElevationMsl(lat, lon)))
            return dted.sourceAt(lat, lon);
        if (!Double.isNaN(fallback.getElevationMsl(lat, lon)))
            return fallback.sourceAt(lat, lon);
        return Double.isNaN(dted.getElevationMsl(lat, lon)) ? "" : dted.sourceAt(lat, lon);
    }
}
