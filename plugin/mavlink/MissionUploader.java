package com.atakmap.android.plugintemplate.plugin.mavlink;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import io.dronefleet.mavlink.common.MavMissionType;
import io.dronefleet.mavlink.common.MissionAck;
import io.dronefleet.mavlink.common.MissionCount;
import io.dronefleet.mavlink.common.MissionItemInt;
import io.dronefleet.mavlink.common.MissionRequest;
import io.dronefleet.mavlink.common.MissionRequestInt;

/**
 * MAVLink mission upload: MISSION_COUNT → (MISSION_REQUEST[_INT] → MISSION_ITEM_INT)* → MISSION_ACK.
 * Blocking; call from a worker thread. Interrupt to cancel.
 */
public class MissionUploader {

    public interface Progress {
        void onItemSent(int seq, int total);
    }

    public static class UploadException extends Exception {
        public UploadException(String msg) {
            super(msg);
        }
    }

    private static final int MAV_MISSION_ACCEPTED = 0;

    private final MavlinkTransport link;
    private final long timeoutMs;
    private final int maxRetries;

    public MissionUploader(MavlinkTransport link) {
        this(link, 1500, 5);
    }

    MissionUploader(MavlinkTransport link, long timeoutMs, int maxRetries) {
        this.link = link;
        this.timeoutMs = timeoutMs;
        this.maxRetries = maxRetries;
    }

    public void upload(List<MissionItemInt> items, Progress progress)
            throws UploadException, InterruptedException {
        final int sys = link.getTargetSystem();
        final BlockingQueue<Object> inbox = new LinkedBlockingQueue<>();
        MavlinkTransport.Listener l = (s, c, p) -> {
            if (s == sys && (p instanceof MissionRequestInt || p instanceof MissionRequest
                    || p instanceof MissionAck))
                inbox.offer(p);
        };
        link.addListener(l);
        try {
            Object lastSent = MissionCount.builder()
                    .targetSystem(sys)
                    .targetComponent(link.getTargetComponent())
                    .count(items.size())
                    .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                    .build();
            link.send(lastSent);
            int retries = 0;

            while (true) {
                Object msg = inbox.poll(timeoutMs, TimeUnit.MILLISECONDS);
                if (msg == null) {
                    if (++retries > maxRetries)
                        throw new UploadException("vehicle stopped responding during upload");
                    link.send(lastSent);
                    continue;
                }
                if (msg instanceof MissionAck) {
                    MissionAck ack = (MissionAck) msg;
                    int type = ack.type().value();
                    if (type == MAV_MISSION_ACCEPTED)
                        return;
                    throw new UploadException("mission rejected: "
                            + (ack.type().entry() != null ? ack.type().entry().name() : type));
                }
                int seq = (msg instanceof MissionRequestInt)
                        ? ((MissionRequestInt) msg).seq()
                        : ((MissionRequest) msg).seq();
                if (seq < 0 || seq >= items.size())
                    throw new UploadException("vehicle requested invalid item " + seq);
                lastSent = items.get(seq);
                link.send(lastSent);
                retries = 0;
                if (progress != null)
                    progress.onItemSent(seq, items.size());
            }
        } finally {
            link.removeListener(l);
        }
    }
}
