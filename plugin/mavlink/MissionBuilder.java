package com.atakmap.android.plugintemplate.plugin.mavlink;

import com.atakmap.android.plugintemplate.plugin.planning.Waypoint;

import java.util.ArrayList;
import java.util.List;

import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.common.MavMissionType;
import io.dronefleet.mavlink.common.MissionItemInt;

/**
 * Builds the ArduPilot mission:
 * <pre>
 *  0  home placeholder (ArduPilot overwrites)
 *  1  NAV_TAKEOFF   relative alt
 *  2… NAV_WAYPOINT  absolute MSL alt (terrain + clearance)
 *  n  NAV_RETURN_TO_LAUNCH
 * </pre>
 */
public final class MissionBuilder {

    private MissionBuilder() {
    }

    public static List<MissionItemInt> build(int sys, int comp, double homeLat, double homeLon,
            double homeAmsl, float takeoffRelAltM, List<Waypoint> waypoints) {
        List<MissionItemInt> items = new ArrayList<>();
        int seq = 0;

        items.add(base(sys, comp, seq++)
                .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                .x(e7(homeLat)).y(e7(homeLon)).z((float) homeAmsl)
                .build());

        items.add(base(sys, comp, seq++)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
                .x(0).y(0).z(takeoffRelAltM)
                .build());

        for (Waypoint wp : waypoints) {
            items.add(base(sys, comp, seq++)
                    .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                    .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                    .x(e7(wp.lat)).y(e7(wp.lon)).z((float) wp.targetAmsl)
                    .build());
        }

        items.add(base(sys, comp, seq)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH)
                .build());
        return items;
    }

    static int e7(double deg) {
        return (int) Math.round(deg * 1e7);
    }

    static MissionItemInt.Builder base(int sys, int comp, int seq) {
        return MissionItemInt.builder()
                .targetSystem(sys)
                .targetComponent(comp)
                .seq(seq)
                .current(0)
                .autocontinue(1)
                .param1(0).param2(0).param3(0).param4(0)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION);
    }
}
