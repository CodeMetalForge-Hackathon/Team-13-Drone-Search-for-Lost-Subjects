package com.atakmap.android.plugintemplate.plugin.mavlink;

/**
 * Minimal abstraction over a MAVLink connection so protocol logic
 * (commands, mission upload) can be unit tested without sockets.
 */
public interface MavlinkTransport {

    interface Listener {
        /** Called on the receive thread for every decoded message. */
        void onMessage(int sysId, int compId, Object payload);
    }

    /** Queue a dronefleet payload object for transmission to the vehicle. */
    void send(Object payload);

    void addListener(Listener l);

    void removeListener(Listener l);

    int getTargetSystem();

    int getTargetComponent();
}
