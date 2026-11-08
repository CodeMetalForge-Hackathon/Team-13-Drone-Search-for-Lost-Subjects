package com.atakmap.android.plugintemplate.plugin.mavlink;

import com.atakmap.android.plugintemplate.plugin.planning.RoutePlanner;
import com.atakmap.android.plugintemplate.plugin.terrain.ElevationSource;
import com.mindfulpigeon.atak.bridge.TerrainLegs;
import com.mindfulpigeon.search.api.Waypoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.common.MissionItemInt;

/**
 * Turns one search route from the search engine (waypoints with heights above ground) into an
 * ArduPilot mission:
 * <pre>
 *  0  home placeholder (ArduPilot overwrites)
 *  1  NAV_TAKEOFF relative to home      — on the ground
 *     NAV_WAYPOINT climb/descend in place — already flying (re-planned route)
 *  2  DO_CHANGE_SPEED: the ground speed the plan's timing assumes
 *  3… NAV_WAYPOINT, absolute MSL altitude
 *  n  NAV_LAND at the route's last point (the recovery point)
 * </pre>
 *
 * <p>Altitudes are terrain-safe for straight legs: each waypoint's MSL altitude is its clearance
 * above ground plus the highest terrain sampled along both legs that touch it. Both ends of every
 * leg are then at least the clearance above every terrain sample on that leg, and ArduPilot flies a
 * leg as a straight line between its ends, so the whole leg keeps the clearance (to the sampling
 * resolution). The first leg starts where the drone climbs to (takeoff, or the climb in place), which
 * is set high enough for it. The price is flying higher than planned over slopes, which the result
 * reports. The altitudes come from {@link TerrainLegs#altitudes}, which the planner also uses to
 * time climbs and descents. Pure Java so it can be unit tested.
 */
public final class SearchMissionBuilder {

    /** Terrain sample spacing along legs, metres (DTED2 post spacing). */
    public static final double SAMPLE_SPACING_M = TerrainLegs.SAMPLE_SPACING_M;
    /** Waypoints per upload; longer routes must be re-planned with a shorter flight time. */
    public static final int MAX_WAYPOINTS = 600;
    /** Consecutive waypoints closer than this are merged (segments repeat their joining point). */
    static final double DUPLICATE_M = 0.5;

    public static final class Result {
        public final List<MissionItemInt> items;
        /** Search waypoints in the mission (after merging duplicates). */
        public final int waypointCount;
        /** NAV_TAKEOFF altitude above home, or NaN when the mission starts in flight. */
        public final float takeoffRelAltM;
        /** Highest waypoint above home, metres. */
        public final double maxAltAboveHomeM;
        /** Largest height above the waypoint's own ground added for terrain on its legs, metres. */
        public final double maxTerrainLiftM;

        Result(List<MissionItemInt> items, int waypointCount, float takeoffRelAltM,
                double maxAltAboveHomeM, double maxTerrainLiftM) {
            this.items = items;
            this.waypointCount = waypointCount;
            this.takeoffRelAltM = takeoffRelAltM;
            this.maxAltAboveHomeM = maxAltAboveHomeM;
            this.maxTerrainLiftM = maxTerrainLiftM;
        }
    }

    private SearchMissionBuilder() {
    }

    /**
     * @param startLat where the drone is now (or the planned launch point when its position is unknown)
     * @param startLon see startLat
     * @param homeAmsl ArduPilot home altitude MSL (NAV_TAKEOFF is relative to it)
     * @param airborne true when the drone is already flying: the mission starts with a climb in place
     *                 instead of a takeoff
     * @param route the route's waypoints in flight order, altitude reference AGL
     * @param terrain terrain heights, MSL
     * @throws RoutePlanner.NoTerrainException when the terrain under a waypoint is unknown
     * @throws IllegalArgumentException for an empty or too long route, or a non-AGL altitude
     */
    public static Result build(int sys, int comp, double startLat, double startLon,
            double homeAmsl, boolean airborne, List<Waypoint> route, ElevationSource terrain)
            throws RoutePlanner.NoTerrainException {
        List<Waypoint> pts = dedupe(route);
        if (pts.isEmpty())
            throw new IllegalArgumentException("the route has no waypoints");
        if (pts.size() > MAX_WAYPOINTS)
            throw new IllegalArgumentException(String.format(Locale.US,
                    "the route has %d waypoints (at most %d per upload): plan a shorter flight",
                    pts.size(), MAX_WAYPOINTS));
        for (Waypoint w : pts) {
            if (!Waypoint.AGL.equals(w.altRef))
                throw new IllegalArgumentException("waypoint altitude reference " + w.altRef
                        + " is not supported (turn terrain following off in the engine config)");
            if (!(w.alt > 0))
                throw new IllegalArgumentException("waypoint height above ground must be positive");
        }

        int n = pts.size();
        // the same terrain-safe altitudes the planner times its climbs and descents with
        double[] alt;
        try {
            alt = TerrainLegs.altitudes(pts, startLat, startLon, terrain::getElevationMsl);
        } catch (TerrainLegs.UnknownTerrainException e) {
            throw new RoutePlanner.NoTerrainException(e.lat, e.lon);
        }
        double maxAboveHome = Double.NEGATIVE_INFINITY;
        double maxLift = 0;
        for (int i = 0; i < n; i++) {
            Waypoint w = pts.get(i);
            maxAboveHome = Math.max(maxAboveHome, alt[i] - homeAmsl);
            maxLift = Math.max(maxLift, alt[i] - w.alt - terrain.getElevationMsl(w.lat, w.lon));
        }

        List<MissionItemInt> items = new ArrayList<>(n + 4);
        int seq = 0;
        items.add(MissionBuilder.base(sys, comp, seq++)
                .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                .x(MissionBuilder.e7(startLat)).y(MissionBuilder.e7(startLon)).z((float) homeAmsl)
                .build());

        float takeoffRel = Float.NaN;
        if (airborne) {
            // climb (or descend) in place to the first leg's safe altitude before heading out
            items.add(MissionBuilder.base(sys, comp, seq++)
                    .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                    .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                    .x(MissionBuilder.e7(startLat)).y(MissionBuilder.e7(startLon))
                    .z((float) alt[0])
                    .build());
        } else {
            takeoffRel = (float) Math.max(pts.get(0).alt, alt[0] - homeAmsl);
            items.add(MissionBuilder.base(sys, comp, seq++)
                    .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                    .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
                    .x(0).y(0).z(takeoffRel)
                    .build());
        }

        double speed = 0;
        for (Waypoint w : pts) {
            if (w.speedMps > 0) {
                speed = w.speedMps;
                break;
            }
        }
        if (speed > 0) {
            items.add(MissionBuilder.base(sys, comp, seq++)
                    .frame(MavFrame.MAV_FRAME_MISSION)
                    .command(MavCmd.MAV_CMD_DO_CHANGE_SPEED)
                    .param1(1) // ground speed
                    .param2((float) speed)
                    .param3(-1) // throttle unchanged
                    .build());
        }

        for (int i = 0; i < n; i++) {
            Waypoint w = pts.get(i);
            items.add(MissionBuilder.base(sys, comp, seq++)
                    .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                    .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                    .x(MissionBuilder.e7(w.lat)).y(MissionBuilder.e7(w.lon)).z((float) alt[i])
                    .build());
        }

        Waypoint last = pts.get(n - 1);
        items.add(MissionBuilder.base(sys, comp, seq)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_LAND)
                .x(MissionBuilder.e7(last.lat)).y(MissionBuilder.e7(last.lon)).z(0)
                .build());

        return new Result(items, n, takeoffRel, maxAboveHome, maxLift);
    }

    /** All segments of a route as one waypoint list, joining points not repeated. */
    public static List<Waypoint> flatten(List<List<Waypoint>> segments) {
        List<Waypoint> out = new ArrayList<>();
        for (List<Waypoint> seg : segments)
            out.addAll(seg);
        return dedupe(out);
    }

    static List<Waypoint> dedupe(List<Waypoint> in) {
        List<Waypoint> out = new ArrayList<>(in.size());
        for (Waypoint w : in) {
            if (!out.isEmpty()) {
                Waypoint p = out.get(out.size() - 1);
                if (RoutePlanner.distance(p.lat, p.lon, w.lat, w.lon) < DUPLICATE_M
                        && p.alt == w.alt)
                    continue;
            }
            out.add(w);
        }
        return out;
    }
}
