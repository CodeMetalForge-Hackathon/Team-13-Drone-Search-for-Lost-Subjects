package com.atakmap.android.plugintemplate.plugin.ui;

import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.atakmap.android.plugintemplate.plugin.R;
import com.atakmap.android.plugintemplate.plugin.mavlink.CopterMode;
import com.atakmap.android.plugintemplate.plugin.mavlink.VehicleState;

import java.util.Locale;

/** Demo tab: GUIDED → arm → takeoff 3 m → 5 ft forward → land. */
public class DemoTabController {

    static final float TAKEOFF_ALT_M = 3.0f;
    static final float FORWARD_M = 1.524f; // 5 ft

    private final Services s;
    private final Button flyBtn;
    private final TextView hintText;
    private final TextView logText;

    public DemoTabController(View root, Services services) {
        this.s = services;
        flyBtn = root.findViewById(R.id.fly_btn);
        hintText = root.findViewById(R.id.demo_hint);
        logText = root.findViewById(R.id.demo_log);

        flyBtn.setOnClickListener(v -> s.confirm("Fly demo?",
                "The drone will arm, take off to 3 m, fly 5 ft (1.5 m) forward, and land."
                        + "\n\nIs the area clear?",
                this::runDemo));
    }

    private void log(String line) {
        Services.appendLog(logText, line);
    }

    private void runDemo() {
        s.flight.started(System.currentTimeMillis());
        s.runner.start("Demo", this::log, ctx -> {
            final VehicleState st = s.link.getState();
            if (st.armed)
                throw new Exception("drone is already armed — start the demo on the ground");

            ctx.log("GUIDED → arming");
            s.commands.setModeBlocking(CopterMode.GUIDED);
            s.commands.armBlocking(true);
            boolean airborne = false;
            try {
                ctx.log("Taking off to " + TAKEOFF_ALT_M + " m");
                s.commands.takeoffBlocking(TAKEOFF_ALT_M);
                airborne = true;
                ctx.waitFor(() -> st.relAlt >= TAKEOFF_ALT_M * 0.9, 20_000, "climb");
                ctx.sleep(1500);

                final boolean haveLocal = st.hasLocalPosition;
                final float x0 = st.localX, y0 = st.localY;
                ctx.log("Moving 5 ft forward");
                s.commands.bodyOffset(FORWARD_M, 0, 0);
                if (haveLocal) {
                    boolean moved = ctx.waitForOptional(() -> Math.hypot(st.localX - x0,
                            st.localY - y0) >= FORWARD_M * 0.85, 10_000);
                    ctx.log(String.format(Locale.US, "%s (%.2f m)",
                            moved ? "Moved" : "Move timed out",
                            Math.hypot(st.localX - x0, st.localY - y0)));
                } else {
                    ctx.sleep(5000);
                }
                ctx.sleep(1500);

                ctx.log("Landing");
                s.commands.setModeBlocking(CopterMode.LAND);
                if (ctx.waitForOptional(() -> !st.armed, 30_000))
                    ctx.log("Landed");
                else
                    ctx.log("Still armed after 30 s — check the drone");
            } catch (InterruptedException e) {
                throw e; // R2Home / Land / KILL took over
            } catch (Exception e) {
                if (airborne) {
                    ctx.log("Error in flight — landing");
                    s.commands.setMode(CopterMode.LAND);
                }
                throw e;
            }
        }, null);
    }

    void refresh(boolean flightActive) {
        boolean connected = s.link.isVehicleConnected();
        boolean enabled = connected && !s.runner.isBusy() && !flightActive;
        flyBtn.setEnabled(enabled);
        flyBtn.setAlpha(enabled ? 1f : 0.45f);
        hintText.setText(!connected ? "Connect a drone first (⚙)"
                : flightActive ? "Flight in progress — use R2Home, Land or KILL above" : "");
    }
}
