package com.atakmap.android.plugintemplate.plugin.search;

import com.atakmap.android.plugintemplate.plugin.terrain.ElevationSource;
import com.atakmap.coremap.maps.conversion.EGM96;
import com.mindfulpigeon.atak.bridge.PluginPorts;

/**
 * The search engine's terrain: the plugin's elevation stack (DTED tiles it downloads or bundles,
 * then whatever ATAK has loaded), converted from MSL to the HAE the engine expects. NaN where no
 * terrain is known. Thread-safe as long as the wrapped source is (BundledDtedSource is).
 */
public final class TerrainElevation implements PluginPorts.Elevation {

    private final ElevationSource msl;

    public TerrainElevation(ElevationSource msl) {
        this.msl = msl;
    }

    @Override
    public double elevationHae(double lat, double lon) {
        double m = msl.getElevationMsl(lat, lon);
        if (Double.isNaN(m))
            return Double.NaN;
        try {
            return EGM96.getHAE(lat, lon, m);
        } catch (RuntimeException | UnsatisfiedLinkError e) {
            return Double.NaN;
        }
    }

    @Override
    public String name() {
        return "P.U.L.S.E. terrain (DTED, then ATAK elevation)";
    }
}
