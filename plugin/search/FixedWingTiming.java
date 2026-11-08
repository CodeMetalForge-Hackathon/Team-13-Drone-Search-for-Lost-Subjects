package com.atakmap.android.plugintemplate.plugin.search;

/**
 * Search-grid timing for a fixed-wing, from flight physics (to be checked against the aircraft's
 * own flight logs). A fixed-wing holds cruise speed through corners, but it cannot reverse into
 * the next lane of the search grid in place: its turn radius r = v^2 / (g tan(bank)) is larger than
 * half the lane spacing s, so the reversal is an omega turn (away, around, back) of length
 * r(pi + 4 theta), with theta = atan2(h, s/2 + r) and h = sqrt(4 r^2 - (s/2 + r)^2). The grid counts a
 * reversal as two 90-degree turns plus one step of s; the extra path, flown at cruise, is charged
 * half per 90 degrees.
 */
public final class FixedWingTiming {

    static final double G = 9.80665;

    private FixedWingTiming() {
    }

    /** Turn radius at a speed (m/s) and bank angle (degrees), metres. */
    public static double turnRadiusM(double speedMps, double bankDeg) {
        return speedMps * speedMps / (G * Math.tan(Math.toRadians(bankDeg)));
    }

    /** Path length of a 180-degree reversal into the lane laneM away, metres. */
    public static double reversalPathM(double speedMps, double bankDeg, double laneM) {
        double r = turnRadiusM(speedMps, bankDeg);
        if (laneM >= 2 * r) {
            return Math.PI * r + (laneM - 2 * r); // two quarter turns joined by a straight
        }
        double a = laneM / 2 + r;
        double h = Math.sqrt(4 * r * r - a * a);
        double theta = Math.atan2(h, a);
        return r * (Math.PI + 4 * theta);
    }

    /** Seconds per 90 degrees of turning on a grid with lanes laneM apart. */
    public static double turnPenaltyS(double speedMps, double bankDeg, double laneM) {
        return (reversalPathM(speedMps, bankDeg, laneM) - laneM) / speedMps / 2;
    }
}
