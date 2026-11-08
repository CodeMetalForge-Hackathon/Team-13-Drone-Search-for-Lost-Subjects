package com.atakmap.android.plugintemplate.plugin.mavlink;

import android.util.Log;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.GlobalPositionInt;
import io.dronefleet.mavlink.common.LocalPositionNed;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.Statustext;
import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;
import io.dronefleet.mavlink.util.EnumValue;

/**
 * MAVLink over UDP, acting as a ground station. Listens on a local port
 * (14550 by default — ArduPilot's standard GCS output) and replies to
 * whichever address the vehicle's heartbeats arrive from.
 */
public class MavlinkLink implements MavlinkTransport {

    private static final String TAG = "KoalifiedMavlink";

    public static final int DEFAULT_PORT = 14550;
    public static final int GCS_SYSID = 255;
    public static final int GCS_COMPID = 190;
    private static final long HEARTBEAT_TIMEOUT_MS = 3000;

    private static final int MSG_ID_LOCAL_POSITION_NED = 32;
    private static final int MSG_ID_GLOBAL_POSITION_INT = 33;
    private static final int MAV_TYPE_GCS = 6;
    private static final int MAV_AUTOPILOT_INVALID = 8;
    private static final int MAV_MODE_FLAG_SAFETY_ARMED = 128;

    private final CopyOnWriteArrayList<Listener> listeners = new CopyOnWriteArrayList<>();
    private final VehicleState state = new VehicleState();

    private final Object txLock = new Object();
    private final ByteArrayOutputStream txBuffer = new ByteArrayOutputStream(512);
    private final MavlinkConnection txConn = MavlinkConnection.create(
            new ByteArrayInputStream(new byte[0]), txBuffer);

    private volatile DatagramSocket socket;
    private Thread rxThread;
    private ScheduledExecutorService scheduler;
    private ExecutorService txExecutor;

    private volatile InetSocketAddress vehicleAddress;
    private volatile int targetSystem = 1;
    private volatile int targetComponent = 1;
    private volatile long lastVehicleHeartbeat;
    private volatile int port = DEFAULT_PORT;

    public synchronized void start(int port) throws SocketException {
        stop();
        DatagramSocket s = new DatagramSocket(null);
        s.setReuseAddress(true);
        s.setBroadcast(true);
        s.bind(new InetSocketAddress(port));
        this.socket = s;
        this.port = port;

        txExecutor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "KoalifiedMavlinkTx");
            t.setDaemon(true);
            return t;
        });
        scheduler = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "KoalifiedMavlinkHb");
            t.setDaemon(true);
            return t;
        });
        scheduler.scheduleAtFixedRate(this::sendGcsHeartbeat, 0, 1, TimeUnit.SECONDS);

        rxThread = new Thread(() -> rxLoop(s), "KoalifiedMavlinkRx");
        rxThread.setDaemon(true);
        rxThread.start();
        Log.i(TAG, "listening on UDP " + port);
    }

    public synchronized void stop() {
        DatagramSocket s = socket;
        socket = null;
        if (s != null)
            s.close();
        if (rxThread != null) {
            rxThread.interrupt();
            rxThread = null;
        }
        if (scheduler != null) {
            scheduler.shutdownNow();
            scheduler = null;
        }
        if (txExecutor != null) {
            txExecutor.shutdown();
            txExecutor = null;
        }
        vehicleAddress = null;
        lastVehicleHeartbeat = 0;
        state.reset();
    }

    public boolean isRunning() {
        DatagramSocket s = socket;
        return s != null && !s.isClosed();
    }

    public int getPort() {
        return port;
    }

    public boolean isVehicleConnected() {
        return vehicleAddress != null
                && System.currentTimeMillis() - lastVehicleHeartbeat < HEARTBEAT_TIMEOUT_MS;
    }

    public InetSocketAddress getVehicleAddress() {
        return vehicleAddress;
    }

    public VehicleState getState() {
        return state;
    }

    @Override
    public int getTargetSystem() {
        return targetSystem;
    }

    @Override
    public int getTargetComponent() {
        return targetComponent;
    }

    @Override
    public void addListener(Listener l) {
        listeners.addIfAbsent(l);
    }

    @Override
    public void removeListener(Listener l) {
        listeners.remove(l);
    }

    @Override
    public void send(Object payload) {
        ExecutorService ex = txExecutor;
        if (ex == null) {
            Log.w(TAG, "send while link stopped: " + payload.getClass().getSimpleName());
            return;
        }
        try {
            ex.execute(() -> sendNow(payload));
        } catch (RejectedExecutionException e) {
            Log.w(TAG, "send rejected (link stopping)");
        }
    }

    private void sendNow(Object payload) {
        DatagramSocket s = socket;
        InetSocketAddress addr = vehicleAddress;
        if (s == null || addr == null) {
            Log.w(TAG, "no vehicle address yet; dropping " + payload.getClass().getSimpleName());
            return;
        }
        byte[] bytes;
        synchronized (txLock) {
            txBuffer.reset();
            try {
                txConn.send2(GCS_SYSID, GCS_COMPID, payload);
            } catch (IOException e) {
                Log.e(TAG, "serialize failed", e);
                return;
            }
            bytes = txBuffer.toByteArray();
        }
        try {
            s.send(new DatagramPacket(bytes, bytes.length, addr));
        } catch (IOException e) {
            Log.w(TAG, "udp send failed: " + e.getMessage());
        }
    }

    private void sendGcsHeartbeat() {
        if (vehicleAddress == null)
            return;
        sendNow(Heartbeat.builder()
                .type(MavType.MAV_TYPE_GCS)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_INVALID)
                .baseMode(EnumValue.<MavModeFlag>create(0))
                .customMode(0)
                .systemStatus(MavState.MAV_STATE_ACTIVE)
                .mavlinkVersion(3)
                .build());
    }

    private void requestStreams() {
        requestInterval(MSG_ID_GLOBAL_POSITION_INT, 200_000);
        requestInterval(MSG_ID_LOCAL_POSITION_NED, 200_000);
    }

    private void requestInterval(int msgId, int intervalUs) {
        send(CommandLong.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .command(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL)
                .param1(msgId)
                .param2(intervalUs)
                .build());
    }

    private void rxLoop(DatagramSocket s) {
        DatagramInputStream in = new DatagramInputStream(s);
        MavlinkConnection conn = MavlinkConnection.create(in, new OutputStream() {
            @Override
            public void write(int b) {
            }
        });
        while (!s.isClosed()) {
            try {
                MavlinkMessage<?> msg = conn.next();
                if (msg == null)
                    break;
                handle(msg, in.getLastSender());
            } catch (IOException e) {
                if (s.isClosed())
                    break;
                Log.w(TAG, "rx error: " + e.getMessage());
            } catch (RuntimeException e) {
                Log.w(TAG, "bad packet: " + e);
            }
        }
        Log.i(TAG, "rx loop exit");
    }

    private void handle(MavlinkMessage<?> msg, InetSocketAddress sender) {
        Object p = msg.getPayload();
        int sys = msg.getOriginSystemId();
        int comp = msg.getOriginComponentId();

        if (p instanceof Heartbeat) {
            Heartbeat hb = (Heartbeat) p;
            if (isVehicle(hb) && (vehicleAddress == null || !isVehicleConnected()
                    || sys == targetSystem)) {
                boolean newlyConnected = !isVehicleConnected();
                targetSystem = sys;
                targetComponent = comp;
                vehicleAddress = sender;
                lastVehicleHeartbeat = System.currentTimeMillis();
                state.armed = (hb.baseMode().value() & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                state.customMode = hb.customMode();
                state.mavType = hb.type().value();
                if (newlyConnected) {
                    Log.i(TAG, "vehicle sys=" + sys + " comp=" + comp + " @ " + sender);
                    requestStreams();
                }
            }
        } else if (sys == targetSystem) {
            if (p instanceof GlobalPositionInt) {
                GlobalPositionInt g = (GlobalPositionInt) p;
                state.lat = g.lat() / 1e7;
                state.lon = g.lon() / 1e7;
                state.altAmsl = g.alt() / 1000.0;
                state.relAlt = g.relativeAlt() / 1000.0;
                state.hasGlobalPosition = g.lat() != 0 || g.lon() != 0;
            } else if (p instanceof LocalPositionNed) {
                LocalPositionNed l = (LocalPositionNed) p;
                state.localX = l.x();
                state.localY = l.y();
                state.localZ = l.z();
                state.hasLocalPosition = true;
            } else if (p instanceof Statustext) {
                state.lastStatusText = ((Statustext) p).text();
            }
        }

        for (Listener l : listeners) {
            try {
                l.onMessage(sys, comp, p);
            } catch (RuntimeException e) {
                Log.w(TAG, "listener failed", e);
            }
        }
    }

    private static boolean isVehicle(Heartbeat hb) {
        return hb.type().value() != MAV_TYPE_GCS
                && hb.autopilot().value() != MAV_AUTOPILOT_INVALID;
    }
}
