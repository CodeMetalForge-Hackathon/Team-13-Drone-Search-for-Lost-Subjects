package com.atakmap.android.plugintemplate.plugin.search;

import com.mindfulpigeon.atak.bridge.DroneProfile;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * The drone types the operator picks from in the Search tab. Endurance is the time in the air on
 * one battery or fuel load; the planner flies each drone for the search time or its endurance,
 * whichever is shorter, and Next flight continues after a landing.
 *
 * <ul>
 * <li>ArduCopter quadcopter: what the plugin's MAVLink link flies. Speed, turns, takeoff and landing
 * fitted in ArduPilot SITL ({@link ArduCopterTiming}); 25 min endurance is a typical small-quad
 * battery, to be set to the airframe's own figure. It searches at 60 m, the height its timing was
 * fitted at, where its camera sees a 104 m swath: 3 cells of 24 m, lanes 72 m apart.</li>
 * <li>EagleNXT eBee TAC: Group 1, fixed-wing (hand-launched, belly-landed), up to 90 min; cruise
 * 11-30 m/s per EagleNXT, planned at 14 m/s. It searches at 400 ft above ground, where its camera
 * sees a 155 m swath: 5 cells of 24 m, lanes 120 m apart.</li>
 * <li>Redwire Stalker Block 30: Group 2 fixed-wing, per the team's figures: 4.9 m (16 ft) wingspan,
 * 22 kg (49 lb) maximum takeoff weight, ~500 ft ceiling, 4 h endurance, 30+ kn cruise (planned at
 * 30 kn, 15.4 m/s), 50+ kn top speed. It searches at 400 ft above ground, under its ceiling, where
 * its camera sees a wide swath: the planner looks at every whole grid cell of the swath at once
 * (5 cells of 24 m) and spaces its lanes as far apart, so it does not fly over each cell.</li>
 * </ul>
 *
 * Each type's swath is its camera's field of view across the track at its search height,
 * {@link #swathM}; the planner looks at every whole grid cell of it at once and spaces its lanes
 * as far apart. The fields of view are planning assumptions: set them to the payloads' figures.
 *
 * <p>Fixed-wing turns come from {@link FixedWingTiming} at a 30 degree bank, for the type's own lane
 * spacing. Their launch, recovery, cruise, climb and descent figures (eBee 3 m/s, Stalker 2.5 m/s)
 * are planning assumptions, not flight-tested here, so they keep a 5% margin; the plugin cannot
 * upload to them (their autopilots are not ArduPilot): their routes are plans to fly from their own
 * ground stations.
 */
public final class DroneTypes {

    /**
     * The quad's search altitude, the plugin's (PluginConfig.searchAltitudeAglM): its takeoff and
     * landing were fitted in SITL climbing to and descending from this height.
     */
    static final double QUAD_SEARCH_AGL_M = 60;
    /**
     * Quad camera's field of view across the track, degrees: a planning assumption for a typical
     * small-quad camera (set it to the airframe's own). At 60 m it sees a 104 m swath.
     */
    static final double QUAD_CAMERA_HFOV_DEG = 82;
    /** Search grid cell (PluginConfig.cellSizeM). */
    static final double CELL_M = 24;
    static final double FIXED_WING_BANK_DEG = 30;
    static final double FIXED_WING_MARGIN = 0.05;
    static final double M_PER_FT = 0.3048;

    /** Group 1 search altitude: 400 ft above ground, the eBee's usual operating height. */
    static final double GROUP1_SEARCH_AGL_M = 400 * M_PER_FT;
    /**
     * Group 1 camera's field of view across the track, degrees: a planning assumption for a wide
     * RGB mapping camera (set it to the payload's figure). At 400 ft it sees a 155 m swath.
     */
    static final double GROUP1_CAMERA_HFOV_DEG = 65;

    /** Group 2 search altitude: 400 ft above ground, 100 ft under its ~500 ft ceiling. */
    static final double GROUP2_SEARCH_AGL_M = 400 * M_PER_FT;
    /**
     * Group 2 camera's field of view across the track, degrees: a planning assumption for an EO
     * gimbal at its wide setting (set it to the payload's figure). At 400 ft it sees a 127 m swath.
     */
    static final double GROUP2_CAMERA_HFOV_DEG = 55;

    /**
     * Its route starts when the takeoff ends; accelerating out of the takeoff hover and stopping
     * over the recovery point are counted with the landing (as ArduCopterTiming.apply keeps them).
     */
    public static final DroneProfile ARDUCOPTER = withMultirotorCamera(new DroneProfile("ArduCopter quadcopter",
            "Multirotor (SITL-calibrated)", 25 * 60, ArduCopterTiming.CRUISE_MPS,
            ArduCopterTiming.TURN_PENALTY_S, ArduCopterTiming.takeoffS(QUAD_SEARCH_AGL_M),
            ArduCopterTiming.landingS(QUAD_SEARCH_AGL_M) + 2 * ArduCopterTiming.TURN_PENALTY_S,
            ArduCopterTiming.MARGIN_FRACTION, true)
            .withVerticalRates(ArduCopterTiming.CLIMB_MPS, ArduCopterTiming.DESCENT_MPS),
            QUAD_SEARCH_AGL_M, swathM(QUAD_SEARCH_AGL_M, QUAD_CAMERA_HFOV_DEG));

    public static final DroneProfile EBEE_TAC = withCamera(
            fixedWing("EagleNXT eBee TAC", "Group 1 fixed-wing", 90 * 60, 14.0, 45, 120, 3.0, 3.0),
            GROUP1_SEARCH_AGL_M, swathM(GROUP1_SEARCH_AGL_M, GROUP1_CAMERA_HFOV_DEG));

    public static final DroneProfile STALKER_BLOCK_30 = withCamera(
            fixedWing("Redwire Stalker Block 30", "Group 2 fixed-wing", 4 * 3600, knots(30), 60, 180,
                    2.5, 2.5),
            GROUP2_SEARCH_AGL_M, swathM(GROUP2_SEARCH_AGL_M, GROUP2_CAMERA_HFOV_DEG));

    public static final List<DroneProfile> ALL = Collections.unmodifiableList(
            Arrays.asList(ARDUCOPTER, EBEE_TAC, STALKER_BLOCK_30));

    private DroneTypes() {
    }

    static double knots(double kn) {
        return kn * 1852 / 3600;
    }

    /** Ground width a camera of this field of view sees from this altitude, looking straight down. */
    static double swathM(double altitudeM, double fovDeg) {
        return 2 * altitudeM * Math.tan(Math.toRadians(fovDeg / 2));
    }

    /**
     * A fixed-wing type: launch and recovery in seconds, cruise, climb and descent in m/s. Its lane
     * change is an omega turn into the next 24 m lane ({@link FixedWingTiming#reversalPathM}).
     */
    static DroneProfile fixedWing(String name, String kind, double enduranceS, double cruiseMps,
            double launchS, double recoveryS, double climbMps, double descentMps) {
        return new DroneProfile(name, kind, enduranceS, cruiseMps,
                FixedWingTiming.turnPenaltyS(cruiseMps, FIXED_WING_BANK_DEG, CELL_M), launchS,
                recoveryS, FIXED_WING_MARGIN, false)
                .withVerticalRates(climbMps, descentMps)
                .withLaneChangePath(FixedWingTiming.reversalPathM(cruiseMps, FIXED_WING_BANK_DEG, CELL_M));
    }

    /**
     * A multirotor with a camera: it searches at that altitude, sees the swath, and flies the
     * connector into lanes as far apart as the whole cells of the swath itself, slowing to turn at
     * both ends.
     */
    static DroneProfile withMultirotorCamera(DroneProfile multirotor, double altitudeM, double swathM) {
        DroneProfile p = multirotor.withCamera(altitudeM, swathM);
        return p.withLaneChangePath(p.laneSpacingM(CELL_M));
    }

    /**
     * A fixed-wing type with a camera: it searches at that altitude, sees the swath, and turns into
     * lanes as far apart as the whole cells of the swath (a U-turn once the lanes are wider than
     * its turning circle).
     */
    static DroneProfile withCamera(DroneProfile fixedWing, double altitudeM, double swathM) {
        DroneProfile p = fixedWing.withCamera(altitudeM, swathM);
        double laneM = p.laneSpacingM(CELL_M);
        return p.withTurnPenalty(FixedWingTiming.turnPenaltyS(p.speedMps, FIXED_WING_BANK_DEG, laneM))
                .withLaneChangePath(FixedWingTiming.reversalPathM(p.speedMps, FIXED_WING_BANK_DEG, laneM));
    }
}
