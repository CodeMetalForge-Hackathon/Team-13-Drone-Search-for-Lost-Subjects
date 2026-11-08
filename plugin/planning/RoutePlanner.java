package com.atakmap.android.plugintemplate.plugin.planning;

import com.atakmap.android.plugintemplate.plugin.terrain.ElevationSource;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Splits the great-circle path from start to destination into evenly spaced
 * waypoints (1/3, 2/3, destination) and sets each to a fixed clearance above
 * the terrain at that waypoint. Pure Java so it can be unit tested.
 */
public final class RoutePlanner {

    public static final double DEFAULT_AGL_M = 100.0;
    public static final int NUM_WAYPOINTS = 3;
    static final double EARTH_RADIUS_M = 6371008.8;
    private static final double LEG_SAMPLE_SPACING_M = 25.0;
    private static final int MAX_SAMPLES_PER_LEG = 400;

    public static class NoTerrainException extends Exception {
        public final double lat;
        public final double lon;

        public NoTerrainException(double lat, double lon) {
            super(String.format(Locale.US,
                    "No terrain data at %.5f, %.5f — bundle a DTED tile for this area", lat, lon));
            this.lat = lat;
            this.lon = lon;
        }
    }

    public static final class Plan {
        public final double startLat;
        public final double startLon;
        /** Terrain at start (MSL), NaN if unknown. */
        public final double startTerrainMsl;
        public final double aglM;
        public final List<Waypoint> waypoints;
        public final double totalDistanceM;
        /**
         * Lowest clearance above sampled terrain anywhere along the straight legs
         * (ArduPilot interpolates altitude linearly between waypoints). NaN if no samples.
         */
        public final double minLegClearanceM;

        Plan(double startLat, double startLon, double startTerrainMsl, double aglM,
                List<Waypoint> waypoints, double totalDistanceM, double minLegClearanceM) {
            this.startLat = startLat;
            this.startLon = startLon;
            this.startTerrainMsl = startTerrainMsl;
            this.aglM = aglM;
            this.waypoints = Collections.unmodifiableList(waypoints);
            this.totalDistanceM = totalDistanceM;
            this.minLegClearanceM = minLegClearanceM;
        }
    }

    private RoutePlanner() {
    }

    public static Plan plan(double startLat, double startLon, double destLat, double destLon,
            ElevationSource elev, double aglM) throws NoTerrainException {
        List<Waypoint> wps = new ArrayList<>(NUM_WAYPOINTS);
        for (int i = 1; i <= NUM_WAYPOINTS; i++) {
            double f = (double) i / NUM_WAYPOINTS;
            double[] p = intermediate(startLat, startLon, destLat, destLon, f);
            double terrain = elev.getElevationMsl(p[0], p[1]);
            if (Double.isNaN(terrain))
                throw new NoTerrainException(p[0], p[1]);
            wps.add(new Waypoint(p[0], p[1], terrain, terrain + aglM,
                    elev.sourceAt(p[0], p[1])));
        }

        double startTerrain = elev.getElevationMsl(startLat, startLon);
        double total = distance(startLat, startLon, destLat, destLon);

        // Clearance along legs: start (at its own terrain+agl, or WP1 alt if unknown) → WP1 → WP2 → WP3
        double prevLat = startLat, prevLon = startLon;
        double prevAlt = Double.isNaN(startTerrain) ? wps.get(0).targetAmsl : startTerrain + aglM;
        double minClear = Double.NaN;
        for (Waypoint wp : wps) {
            double legLen = distance(prevLat, prevLon, wp.lat, wp.lon);
            int n = (int) Math.min(MAX_SAMPLES_PER_LEG,
                    Math.max(2, Math.ceil(legLen / LEG_SAMPLE_SPACING_M)));
            for (int k = 0; k <= n; k++) {
                double f = (double) k / n;
                double[] p = intermediate(prevLat, prevLon, wp.lat, wp.lon, f);
                double t = elev.getElevationMsl(p[0], p[1]);
                if (Double.isNaN(t))
                    continue;
                double alt = prevAlt + (wp.targetAmsl - prevAlt) * f;
                double c = alt - t;
                if (Double.isNaN(minClear) || c < minClear)
                    minClear = c;
            }
            prevLat = wp.lat;
            prevLon = wp.lon;
            prevAlt = wp.targetAmsl;
        }

        return new Plan(startLat, startLon, startTerrain, aglM, wps, total, minClear);
    }

    /** Points every ~step along the path, start and end included ({lat, lon} each). */
    public static List<double[]> samplePath(double lat1, double lon1, double lat2, double lon2,
            double stepM) {
        double d = distance(lat1, lon1, lat2, lon2);
        int n = (int) Math.max(1, Math.min(2000, Math.ceil(d / stepM)));
        List<double[]> out = new ArrayList<>(n + 1);
        for (int k = 0; k <= n; k++)
            out.add(intermediate(lat1, lon1, lat2, lon2, (double) k / n));
        return out;
    }

    /** Great-circle distance in metres. */
    public static double distance(double lat1, double lon1, double lat2, double lon2) {
        double p1 = Math.toRadians(lat1), p2 = Math.toRadians(lat2);
        double dp = p2 - p1, dl = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dp / 2) * Math.sin(dp / 2)
                + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) * Math.sin(dl / 2);
        return 2 * EARTH_RADIUS_M * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    /** Point at fraction f (0..1) along the great circle from 1 to 2. Returns {lat, lon}. */
    public static double[] intermediate(double lat1, double lon1, double lat2, double lon2,
            double f) {
        double p1 = Math.toRadians(lat1), l1 = Math.toRadians(lon1);
        double p2 = Math.toRadians(lat2), l2 = Math.toRadians(lon2);
        double d = distance(lat1, lon1, lat2, lon2) / EARTH_RADIUS_M;
        if (d < 1e-12)
            return new double[] {lat1, lon1};
        double a = Math.sin((1 - f) * d) / Math.sin(d);
        double b = Math.sin(f * d) / Math.sin(d);
        double x = a * Math.cos(p1) * Math.cos(l1) + b * Math.cos(p2) * Math.cos(l2);
        double y = a * Math.cos(p1) * Math.sin(l1) + b * Math.cos(p2) * Math.sin(l2);
        double z = a * Math.sin(p1) + b * Math.sin(p2);
        double lat = Math.atan2(z, Math.sqrt(x * x + y * y));
        double lon = Math.atan2(y, x);
        return new double[] {Math.toDegrees(lat), Math.toDegrees(lon)};
    }
}
