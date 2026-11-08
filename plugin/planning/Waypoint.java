package com.atakmap.android.plugintemplate.plugin.planning;

/** A planned waypoint: position, terrain height and commanded altitude (both MSL, metres). */
public final class Waypoint {
    public final double lat;
    public final double lon;
    public final double terrainMsl;
    public final double targetAmsl;
    /** Which terrain data produced terrainMsl, e.g. "DTED2", "DTED0", "ATAK". */
    public final String terrainSource;

    public Waypoint(double lat, double lon, double terrainMsl, double targetAmsl) {
        this(lat, lon, terrainMsl, targetAmsl, "");
    }

    public Waypoint(double lat, double lon, double terrainMsl, double targetAmsl,
            String terrainSource) {
        this.lat = lat;
        this.lon = lon;
        this.terrainMsl = terrainMsl;
        this.targetAmsl = targetAmsl;
        this.terrainSource = terrainSource == null ? "" : terrainSource;
    }

    /** True when terrain came from coarse (~900 m) DTED0 data. */
    public boolean isCoarseTerrain() {
        return "DTED0".equals(terrainSource);
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "%.6f, %.6f  terrain %.0f m (%s)  alt %.0f m MSL",
                lat, lon, terrainMsl, terrainSource, targetAmsl);
    }
}
