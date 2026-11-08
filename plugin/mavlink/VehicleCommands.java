package com.atakmap.android.plugintemplate.plugin.mavlink;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.common.MissionCurrent;
import io.dronefleet.mavlink.common.MissionSetCurrent;
import io.dronefleet.mavlink.common.PositionTargetTypemask;
import io.dronefleet.mavlink.common.SetPositionTargetLocalNed;
import io.dronefleet.mavlink.util.EnumValue;

/**
 * Vehicle commands. Methods named *Blocking wait for COMMAND_ACK and must
 * be called from a worker thread; they throw on rejection or timeout and
 * honour thread interruption (used for cancel).
 */
public class VehicleCommands {

    /** Magic param2 for MAV_CMD_COMPONENT_ARM_DISARM: force disarm even in flight. */
    public static final float FORCE_DISARM_MAGIC = 21196f;

    private static final int MAV_RESULT_ACCEPTED = 0;
    private static final int MAV_RESULT_IN_PROGRESS = 5;
    private static final long ACK_TIMEOUT_MS = 1500;
    private static final int ATTEMPTS = 3;

    /** Ignore velocity, acceleration, yaw and yaw rate: position-only target. */
    static final int TYPE_MASK_POSITION_ONLY = 0x0FF8;

    public static class CommandFailedException extends Exception {
        public CommandFailedException(String msg) {
            super(msg);
        }
    }

    private final MavlinkTransport link;

    public VehicleCommands(MavlinkTransport link) {
        this.link = link;
    }

    // ---- blocking (worker thread) -----------------------------------------

    public void setModeBlocking(int customMode)
            throws CommandFailedException, InterruptedException {
        commandBlocking(MavCmd.MAV_CMD_DO_SET_MODE, 1f /* custom mode enabled */,
                customMode, 0, 0, 0, 0, 0);
    }

    public void armBlocking(boolean arm)
            throws CommandFailedException, InterruptedException {
        commandBlocking(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1f : 0f,
                0, 0, 0, 0, 0, 0);
    }

    public void takeoffBlocking(float relAltM)
            throws CommandFailedException, InterruptedException {
        commandBlocking(MavCmd.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, Float.NaN, 0, 0, relAltM);
    }

    public void missionStartBlocking()
            throws CommandFailedException, InterruptedException {
        commandBlocking(MavCmd.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0);
    }

    /**
     * Makes mission item {@code seq} the one AUTO flies next, confirmed by the vehicle:
     * MAV_CMD_DO_SET_MISSION_CURRENT (acknowledged; ArduPilot 4.4+), else the older
     * MISSION_SET_CURRENT message, confirmed by the MISSION_CURRENT the vehicle sends back.
     */
    public void setMissionCurrentBlocking(int seq)
            throws CommandFailedException, InterruptedException {
        try {
            commandBlocking(MavCmd.MAV_CMD_DO_SET_MISSION_CURRENT, seq, 0, 0, 0, 0, 0, 0);
            return;
        } catch (CommandFailedException e) {
            // older firmware: fall back to the message
        }
        final BlockingQueue<Integer> current = new LinkedBlockingQueue<>();
        MavlinkTransport.Listener l = (sys, comp, payload) -> {
            if (sys == link.getTargetSystem() && payload instanceof MissionCurrent)
                current.offer(((MissionCurrent) payload).seq());
        };
        link.addListener(l);
        try {
            for (int attempt = 0; attempt < ATTEMPTS; attempt++) {
                link.send(MissionSetCurrent.builder()
                        .targetSystem(link.getTargetSystem())
                        .targetComponent(link.getTargetComponent())
                        .seq(seq)
                        .build());
                long deadline = System.currentTimeMillis() + ACK_TIMEOUT_MS;
                long remaining;
                while ((remaining = deadline - System.currentTimeMillis()) > 0) {
                    Integer c = current.poll(remaining, TimeUnit.MILLISECONDS);
                    if (c == null)
                        break;
                    if (c == seq)
                        return;
                }
            }
            throw new CommandFailedException("SET_MISSION_CURRENT: vehicle did not confirm item "
                    + seq);
        } finally {
            link.removeListener(l);
        }
    }

    /**
     * Send a COMMAND_LONG and wait for its ACK, retrying (with incremented
     * confirmation) on timeout.
     */
    public void commandBlocking(MavCmd cmd, float p1, float p2, float p3, float p4,
            float p5, float p6, float p7)
            throws CommandFailedException, InterruptedException {
        final BlockingQueue<CommandAck> acks = new LinkedBlockingQueue<>();
        MavlinkTransport.Listener l = (sys, comp, payload) -> {
            if (payload instanceof CommandAck
                    && ((CommandAck) payload).command().entry() == cmd)
                acks.offer((CommandAck) payload);
        };
        link.addListener(l);
        try {
            for (int attempt = 0; attempt < ATTEMPTS; attempt++) {
                link.send(commandLong(cmd, attempt, p1, p2, p3, p4, p5, p6, p7));
                long deadline = System.currentTimeMillis() + ACK_TIMEOUT_MS;
                long remaining;
                while ((remaining = deadline - System.currentTimeMillis()) > 0) {
                    CommandAck ack = acks.poll(remaining, TimeUnit.MILLISECONDS);
                    if (ack == null)
                        break;
                    int r = ack.result().value();
                    if (r == MAV_RESULT_ACCEPTED)
                        return;
                    if (r == MAV_RESULT_IN_PROGRESS)
                        continue;
                    throw new CommandFailedException(shortName(cmd) + " rejected: "
                            + resultName(r));
                }
            }
            throw new CommandFailedException(shortName(cmd) + ": no ACK from vehicle");
        } finally {
            link.removeListener(l);
        }
    }

    // ---- fire-and-forget (any thread) -------------------------------------

    public void setMode(int customMode) {
        link.send(commandLong(MavCmd.MAV_CMD_DO_SET_MODE, 0, 1f, customMode,
                0, 0, 0, 0, 0));
    }

    /** Emergency motor stop. Drops the vehicle out of the sky if airborne. */
    public void forceDisarm() {
        link.send(commandLong(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0f,
                FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0));
    }

    /**
     * Relative move in the vehicle body frame (x forward, y right, z down),
     * metres. Vehicle must be in GUIDED.
     */
    public void bodyOffset(float forwardM, float rightM, float downM) {
        link.send(SetPositionTargetLocalNed.builder()
                .timeBootMs(0)
                .targetSystem(link.getTargetSystem())
                .targetComponent(link.getTargetComponent())
                .coordinateFrame(MavFrame.MAV_FRAME_BODY_OFFSET_NED)
                .typeMask(EnumValue.<PositionTargetTypemask>create(TYPE_MASK_POSITION_ONLY))
                .x(forwardM)
                .y(rightM)
                .z(downM)
                .build());
    }

    private CommandLong commandLong(MavCmd cmd, int confirmation, float p1, float p2,
            float p3, float p4, float p5, float p6, float p7) {
        return CommandLong.builder()
                .targetSystem(link.getTargetSystem())
                .targetComponent(link.getTargetComponent())
                .command(cmd)
                .confirmation(confirmation)
                .param1(p1).param2(p2).param3(p3).param4(p4)
                .param5(p5).param6(p6).param7(p7)
                .build();
    }

    private static String shortName(MavCmd cmd) {
        return cmd.name().replace("MAV_CMD_", "");
    }

    private static String resultName(int r) {
        switch (r) {
            case 1: return "TEMPORARILY_REJECTED";
            case 2: return "DENIED";
            case 3: return "UNSUPPORTED";
            case 4: return "FAILED";
            case 6: return "CANCELLED";
            default: return "result " + r;
        }
    }
}
