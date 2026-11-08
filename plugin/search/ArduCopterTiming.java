package com.atakmap.android.plugintemplate.plugin.search;

import com.mindfulpigeon.atak.bridge.PluginConfig;

/**
 * How long ArduCopter takes to fly a search route, for the search planner's timing.
 *
 * <p>Fitted to ArduPilot SITL flights of planned search routes (sim/run-sitl-test.sh, ArduCopter
 * with default WPNAV / LAND parameters, 2026-11-07): over 44 flown legs, leg time =
 * length / 10.1 m/s + 2.8 s per 90 degrees of turning, mean error 0.2 s per leg. The drone never
 * passed about 10 m/s (ArduCopter's default waypoint speed) even when 12 m/s was commanded.
 * Takeoff to 60 m took 30.6 s and the landing from 60 m 58.8 s; both are modeled from ArduCopter's
 * climb and descent speeds plus the measured overheads.
 *
 * <p>Re-check on the real airframe with the same harness; a vehicle with other WPNAV settings
 * needs its own numbers.
 */
public final class ArduCopterTiming {

    /** Straight-leg cruise speed, m/s (also sent as the mission's DO_CHANGE_SPEED). */
    public static final double CRUISE_MPS = 10.0;
    /** Time a 90 degree turn costs, s (a reversal costs twice that). */
    public static final double TURN_PENALTY_S = 2.8;

    /**
     * Seconds a re-planned route may take to start: planning (5 s deadline), the operator's review,
     * the hold and the upload, and flying back to where the drone was when Re-plan was pressed.
     */
    public static final double REPLAN_ALLOWANCE_S = 30;
    /** Stopping from cruise into the GUIDED hold and uploading the new mission, s. */
    public static final double HOLD_AND_UPLOAD_S = 5;
    /**
     * Share of the flight time kept as a safety margin. Routes fill their time; in SITL a
     * 1200 s coverage flight with no re-plan landed 2 s late on a model that was exact on its legs.
     */
    public static final double MARGIN_FRACTION = 0.02;

    // takeoff: WPNAV_SPEED_UP 2.5 m/s, plus spool-up and the speed change (fitted)
    /** Fastest climb, m/s (WPNAV_SPEED_UP): also limits climbing legs over terrain. */
    public static final double CLIMB_MPS = 2.5;
    static final double CLIMB_OVERHEAD_S = 6.6;
    // landing: WPNAV_SPEED_DN 1.5 m/s down to LAND_ALT_LOW 10 m, then LAND_SPEED 0.5 m/s (fitted overhead)
    /** Fastest descent, m/s (WPNAV_SPEED_DN): also limits descending legs over terrain. */
    public static final double DESCENT_MPS = 1.5;
    static final double LAND_FINAL_ALT_M = 10;
    static final double LAND_FINAL_MPS = 0.5;
    static final double LAND_OVERHEAD_S = 5.5;

    private ArduCopterTiming() {
    }

    /** Takeoff to altM above home, s. */
    public static double takeoffS(double altM) {
        return altM / CLIMB_MPS + CLIMB_OVERHEAD_S;
    }

    /** Landing from altM above ground, s. */
    public static double landingS(double altM) {
        double fast = Math.max(0, altM - LAND_FINAL_ALT_M);
        double slow = Math.min(altM, LAND_FINAL_ALT_M);
        return fast / DESCENT_MPS + slow / LAND_FINAL_MPS + LAND_OVERHEAD_S;
    }

    /**
     * Applies the timing to a search config: cruise speed, turn penalty, the flight time kept for
     * takeoff, landing, accelerating out of the takeoff hover and stopping over the recovery point
     * (at the config's search altitude), and the re-plan start allowance.
     */
    public static void apply(PluginConfig cfg) {
        cfg.speedMps = CRUISE_MPS;
        cfg.turnPenaltyS = TURN_PENALTY_S;
        double alt = cfg.searchAltitudeAglM;
        cfg.takeoffLandingS = takeoffS(alt) + landingS(alt) + 2 * TURN_PENALTY_S;
        cfg.takeoffS = takeoffS(alt);
        cfg.replanStartAllowanceS = REPLAN_ALLOWANCE_S;
        cfg.flightTimeMarginFraction = MARGIN_FRACTION;
        cfg.climbMps = CLIMB_MPS;
        cfg.descentMps = DESCENT_MPS;
    }

    /**
     * How many seconds a re-planned route would overrun its flight time if uploaded now; 0 or less
     * means it still fits. The route was sized at Re-plan for its flight time minus the allowance.
     * Since then the drone kept flying the old route: the seconds that passed, the flight back to the
     * route's start (where the drone was at Re-plan) and the hold and upload all come out of it.
     *
     * @param secondsSinceReplan time since Re-plan was pressed
     * @param metresToRouteStart distance from the drone now to the new route's first waypoint
     * @param allowanceS the allowance the route was sized with
     */
    public static double replanOverrunS(double secondsSinceReplan, double metresToRouteStart,
            double allowanceS) {
        return secondsSinceReplan + metresToRouteStart / CRUISE_MPS + HOLD_AND_UPLOAD_S
                - allowanceS;
    }
}
