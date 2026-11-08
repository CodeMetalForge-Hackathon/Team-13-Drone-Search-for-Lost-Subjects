package com.atakmap.android.plugintemplate.plugin.terrain;

/** Terrain height lookup. */
public interface ElevationSource {
    /** @return terrain height above mean sea level in metres, or NaN if unknown. */
    double getElevationMsl(double lat, double lon);

    /** Short label of the data that would answer {@link #getElevationMsl} (e.g. "DTED2"). */
    default String sourceAt(double lat, double lon) {
        return "";
    }
}
