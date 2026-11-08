package com.atakmap.android.plugintemplate.plugin.terrain;

import com.atakmap.coremap.maps.conversion.EGM96;
import com.atakmap.coremap.maps.coords.GeoPointMetaData;
import com.atakmap.map.elevation.ElevationManager;

/** Fallback: whatever elevation data ATAK itself has loaded (returns HAE → converted to MSL). */
public class AtakElevationSource implements ElevationSource {
    @Override
    public double getElevationMsl(double lat, double lon) {
        try {
            double hae = ElevationManager.getElevation(lat, lon, null);
            if (Double.isNaN(hae))
                return Double.NaN;
            return EGM96.getMSL(lat, lon, hae);
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    /** The data ATAK answers from (e.g. "DTED2", "SRTM1", "LIDAR"), or "ATAK" when it does not say. */
    @Override
    public String sourceAt(double lat, double lon) {
        try {
            GeoPointMetaData md = ElevationManager.getElevationMetadata(lat, lon, null);
            String src = md != null ? md.getAltitudeSource() : null;
            if (src != null && !src.trim().isEmpty() && !GeoPointMetaData.UNKNOWN.equals(src))
                return src.trim();
        } catch (RuntimeException ignored) {
        }
        return "ATAK";
    }
}
