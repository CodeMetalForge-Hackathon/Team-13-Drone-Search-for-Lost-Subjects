package com.atakmap.android.plugintemplate.plugin.ui;

/**
 * Decides when in-flight controls (R2Home / Land / KILL) are shown:
 * from the moment a mission or demo is started until the vehicle has armed
 * and then disarmed again. Whenever the vehicle reports armed — for any
 * reason, e.g. armed from the RC — the controls are shown too. A heartbeat
 * dropout keeps the last armed state, so controls stay up during link blips.
 */
public class FlightSession {

    /** Give up waiting for the vehicle to arm after this long. */
    static final long ARM_WAIT_MS = 60_000;

    private boolean active;
    private long startedAt;
    private boolean sawArmed;

    /** Called when the user starts a mission or demo flight. */
    public synchronized void started(long now) {
        active = true;
        startedAt = now;
        sawArmed = false;
    }

    /** Called when the flight session ends explicitly (e.g. disconnect). */
    public synchronized void reset() {
        active = false;
        sawArmed = false;
    }

    /**
     * @param connected whether heartbeats are currently arriving
     * @param armed last known armed state (stale while disconnected)
     * @return whether the in-flight controls should be visible
     */
    public synchronized boolean isActive(boolean connected, boolean armed, long now) {
        if (armed) {
            if (!active) {
                active = true;
                startedAt = now;
            }
            sawArmed = true;
            return true;
        }
        if (!active)
            return false;
        if ((connected && sawArmed) || now - startedAt > ARM_WAIT_MS) {
            reset(); // landed and disarmed, or never armed
            return false;
        }
        return true; // started, waiting for arm
    }
}
