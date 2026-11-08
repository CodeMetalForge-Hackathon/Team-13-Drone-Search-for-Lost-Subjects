package com.atakmap.android.plugintemplate.plugin.ui;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;

import com.atakmap.android.plugintemplate.plugin.R;
import com.atakmap.android.plugintemplate.plugin.mavlink.CopterMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * In-flight controls: R2Home (RTL), Land, and hold-to-fire KILL (force disarm).
 * Shown only while a flight is in progress (see {@link FlightSession}).
 * All three pre-empt any running sequence.
 */
public class SafetyBar {

    private static final long KILL_HOLD_MS = 1000;
    private static final int KILL_REPEATS = 3;
    private static final long KILL_REPEAT_SPACING_MS = 100;

    private final Services s;
    private final Handler main = new Handler(Looper.getMainLooper());
    private final ExecutorService cmdExec = Executors.newSingleThreadExecutor();

    private final View panel;
    private final ProgressBar killProgress;
    private final TextView banner;

    private long killDownAt;
    private boolean killFired;
    private boolean killPending;

    private final Runnable killTick = new Runnable() {
        @Override
        public void run() {
            long held = SystemClock.uptimeMillis() - killDownAt;
            killProgress.setProgress((int) Math.min(100, held * 100 / KILL_HOLD_MS));
            if (held >= KILL_HOLD_MS) {
                killFired = true;
                fireKill();
            } else {
                main.postDelayed(this, 30);
            }
        }
    };

    @SuppressLint("ClickableViewAccessibility")
    public SafetyBar(View root, Services services) {
        this.s = services;
        panel = root.findViewById(R.id.flight_controls);
        killProgress = root.findViewById(R.id.kill_progress);
        banner = root.findViewById(R.id.banner);
        banner.setOnClickListener(v -> banner.setVisibility(View.GONE));

        root.findViewById(R.id.r2home_btn).setOnClickListener(v -> returnHome());
        root.findViewById(R.id.land_btn).setOnClickListener(v -> land());

        Button killBtn = root.findViewById(R.id.kill_btn);
        killBtn.setOnTouchListener((v, ev) -> {
            switch (ev.getActionMasked()) {
                case MotionEvent.ACTION_DOWN:
                    killDownAt = SystemClock.uptimeMillis();
                    killFired = false;
                    v.setPressed(true);
                    main.post(killTick);
                    return true;
                case MotionEvent.ACTION_UP:
                case MotionEvent.ACTION_CANCEL:
                    v.setPressed(false);
                    main.removeCallbacks(killTick);
                    if (!killFired) {
                        killProgress.setProgress(0);
                        if (ev.getActionMasked() == MotionEvent.ACTION_UP)
                            s.toast("Hold KILL for 1 second to stop the motors");
                    }
                    return true;
                default:
                    return true;
            }
        });
    }

    public void returnHome() {
        sendMode(CopterMode.RTL, "Returning home");
    }

    public void land() {
        sendMode(CopterMode.LAND, "Landing");
    }

    private void sendMode(int mode, String label) {
        s.runner.cancelAll();
        if (!s.link.isVehicleConnected()) {
            showBanner("No drone link — command not sent", Color.parseColor("#E65100"));
            return;
        }
        showBanner(label + "…", Color.parseColor("#1565C0"));
        s.commands.setMode(mode); // immediate, then confirm with ACK
        cmdExec.execute(() -> {
            String result;
            int color;
            try {
                s.commands.setModeBlocking(mode);
                result = label;
                color = Color.parseColor("#2E7D32");
            } catch (Exception e) {
                result = label + " failed: " + e.getMessage();
                color = Color.parseColor("#B00020");
            }
            final String r = result;
            final int c = color;
            main.post(() -> showBanner(r, c));
        });
    }

    private void fireKill() {
        s.runner.cancelAll();
        if (!s.link.isVehicleConnected()) {
            showBanner("No drone link — KILL NOT DELIVERED", Color.parseColor("#B00020"));
            return;
        }
        for (int i = 0; i < KILL_REPEATS; i++)
            main.postDelayed(s.commands::forceDisarm, i * KILL_REPEAT_SPACING_MS);
        killPending = true;
        showBanner("KILL sent — stopping motors", Color.parseColor("#B00020"));
    }

    /** Called by the UI ticker. */
    void refresh(boolean flightActive) {
        panel.setVisibility(flightActive ? View.VISIBLE : View.GONE);
        if (killPending && s.link.isVehicleConnected() && !s.link.getState().armed) {
            killPending = false;
            killProgress.setProgress(0);
            showBanner("Motors stopped (drone disarmed)", Color.parseColor("#B00020"));
        }
    }

    void showBanner(String text, int color) {
        banner.setVisibility(View.VISIBLE);
        banner.setBackgroundColor(color);
        banner.setText(text);
    }

    void dispose() {
        main.removeCallbacksAndMessages(null);
        cmdExec.shutdownNow();
    }
}
