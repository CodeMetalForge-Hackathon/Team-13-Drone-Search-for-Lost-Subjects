package com.atakmap.android.plugintemplate.plugin.ui;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

/**
 * Which of a plan's warnings the Search tab shows: only those that say a route breaks its flight
 * time, that a drone gets no route at all, or that a drone takes off from the device's position
 * (no launch point on the map). The rest (the SERE model's notes, estimated map layers, coarse
 * terrain, reserved takeoff and landing time) are not shown.
 */
final class PlanWarnings {

    private static final Pattern SHOWN = Pattern.compile(
            // FleetPlanner: a drone without a route (mostly too little flight time)
            "^drone \\S+: (flight time too short|transit blocked by red zones|no reachable search cell)"
                    // climbs and descents push a route past its time
                    + "|over its route time"
                    // a re-plan that could not be made
                    + "|was not re-planned"
                    + "|^the whole search area lies inside red zones"
                    // nothing on the map says where it takes off
                    + "|launches from the device position");

    private PlanWarnings() {
    }

    static List<String> shown(List<String> warnings) {
        List<String> out = new ArrayList<>();
        for (String w : warnings)
            if (w != null && SHOWN.matcher(w).find())
                out.add(w);
        return out;
    }
}
