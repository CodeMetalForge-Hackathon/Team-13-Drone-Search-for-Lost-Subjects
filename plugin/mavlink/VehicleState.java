package com.atakmap.android.plugintemplate.plugin.mavlink;

/** Latest telemetry snapshot from the vehicle. Written by the receive thread. */
public class VehicleState {

    public volatile boolean armed;
    public volatile long customMode = -1;
    public volatile int mavType = -1;

    public volatile boolean hasGlobalPosition;
    public volatile double lat;
    public volatile double lon;
    /** Altitude above mean sea level, metres. */
    public volatile double altAmsl;
    /** Altitude relative to home, metres. */
    public volatile double relAlt;

    public volatile boolean hasLocalPosition;
    public volatile float localX;
    public volatile float localY;
    public volatile float localZ;

    public volatile String lastStatusText;

    public void reset() {
        armed = false;
        customMode = -1;
        mavType = -1;
        hasGlobalPosition = false;
        hasLocalPosition = false;
        lastStatusText = null;
    }
}
