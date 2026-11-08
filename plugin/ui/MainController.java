package com.atakmap.android.plugintemplate.plugin.ui;

import android.graphics.Color;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.TextView;

import com.atak.plugins.impl.PluginLayoutInflater;
import com.atakmap.android.plugintemplate.plugin.R;
import com.atakmap.android.plugintemplate.plugin.mavlink.CopterMode;
import com.atakmap.android.plugintemplate.plugin.mavlink.VehicleState;

import java.net.InetSocketAddress;
import java.util.Locale;

/** Owns the pane: short status, settings drawer, in-flight controls, tabs and the UI ticker. */
public class MainController {

    private static final long TICK_MS = 500;
    private static final int COLOR_OK = Color.parseColor("#66BB6A");
    private static final int COLOR_WAIT = Color.parseColor("#FFB300");
    private static final int COLOR_OFF = Color.parseColor("#9E9E9E");
    private static final int TAB_SELECTED_BG = Color.parseColor("#424242");

    private final Services s;
    private final View root;
    private final Handler main = new Handler(Looper.getMainLooper());

    private final TextView statusText;
    private final View settingsPanel;
    private final EditText portEdit;
    private final Button connectBtn;
    private final TextView linkDetail;
    private static final int TAB_PLANNING = 0;
    private static final int TAB_SEARCH = 1;
    private static final int TAB_DEMO = 2;

    private final Button tabPlanningBtn;
    private final Button tabSearchBtn;
    private final Button tabDemoBtn;
    private final View planningView;
    private final View searchView;
    private final View demoView;

    private final SafetyBar safety;
    private final PlanningTabController planning;
    private final SearchTabController search;
    private final DemoTabController demo;
    private boolean disposed;

    private final Runnable ticker = new Runnable() {
        @Override
        public void run() {
            if (disposed)
                return;
            refresh();
            main.postDelayed(this, TICK_MS);
        }
    };

    public MainController(Services services) {
        this.s = services;
        root = PluginLayoutInflater.inflate(s.pluginContext, R.layout.main_layout, null);

        statusText = root.findViewById(R.id.status_text);
        settingsPanel = root.findViewById(R.id.settings_panel);
        portEdit = root.findViewById(R.id.port_edit);
        connectBtn = root.findViewById(R.id.connect_btn);
        linkDetail = root.findViewById(R.id.link_detail);
        tabPlanningBtn = root.findViewById(R.id.tab_planning_btn);
        tabSearchBtn = root.findViewById(R.id.tab_search_btn);
        tabDemoBtn = root.findViewById(R.id.tab_demo_btn);

        FrameLayout tabContainer = root.findViewById(R.id.tab_container);
        planningView = PluginLayoutInflater.inflate(s.pluginContext, R.layout.tab_planning, null);
        searchView = PluginLayoutInflater.inflate(s.pluginContext, R.layout.tab_search, null);
        demoView = PluginLayoutInflater.inflate(s.pluginContext, R.layout.tab_demo, null);
        for (View tab : new View[] {planningView, searchView, demoView})
            tabContainer.addView(tab, new FrameLayout.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        safety = new SafetyBar(root, s);
        planning = new PlanningTabController(planningView, s);
        search = new SearchTabController(searchView, s);
        demo = new DemoTabController(demoView, s);

        portEdit.setText(String.valueOf(s.link.getPort()));
        connectBtn.setOnClickListener(v -> toggleConnection());
        root.findViewById(R.id.settings_btn).setOnClickListener(v -> settingsPanel
                .setVisibility(settingsPanel.getVisibility() == View.VISIBLE ? View.GONE
                        : View.VISIBLE));
        statusText.setOnClickListener(v -> settingsPanel.setVisibility(View.VISIBLE));
        tabPlanningBtn.setOnClickListener(v -> selectTab(TAB_PLANNING));
        tabSearchBtn.setOnClickListener(v -> selectTab(TAB_SEARCH));
        tabDemoBtn.setOnClickListener(v -> selectTab(TAB_DEMO));
        selectTab(TAB_PLANNING); // mission planning is the main tab

        main.post(ticker);
    }

    public View getView() {
        return root;
    }

    private void selectTab(int tab) {
        planningView.setVisibility(tab == TAB_PLANNING ? View.VISIBLE : View.GONE);
        searchView.setVisibility(tab == TAB_SEARCH ? View.VISIBLE : View.GONE);
        demoView.setVisibility(tab == TAB_DEMO ? View.VISIBLE : View.GONE);
        styleTab(tabPlanningBtn, tab == TAB_PLANNING);
        styleTab(tabSearchBtn, tab == TAB_SEARCH);
        styleTab(tabDemoBtn, tab == TAB_DEMO);
    }

    private static void styleTab(Button b, boolean selected) {
        b.setBackgroundColor(selected ? TAB_SELECTED_BG : Color.TRANSPARENT);
        b.setTextColor(selected ? Color.WHITE : COLOR_OFF);
    }

    private void toggleConnection() {
        if (s.link.isRunning()) {
            s.runner.cancelAll();
            s.link.stop();
            s.flight.reset();
            return;
        }
        int port;
        try {
            port = Integer.parseInt(portEdit.getText().toString().trim());
            if (port < 1 || port > 65535)
                throw new NumberFormatException();
        } catch (NumberFormatException e) {
            s.toast("Invalid UDP port");
            return;
        }
        try {
            s.link.start(port);
        } catch (Exception e) {
            s.toast("Cannot listen on UDP " + port + ": " + e.getMessage());
        }
    }

    private void refresh() {
        boolean running = s.link.isRunning();
        boolean connected = s.link.isVehicleConnected();
        VehicleState st = s.link.getState();

        // short status line
        if (!running) {
            statusText.setText("○ Offline — tap to connect");
            statusText.setTextColor(COLOR_OFF);
        } else if (!connected) {
            statusText.setText("● Waiting for drone…");
            statusText.setTextColor(COLOR_WAIT);
        } else {
            StringBuilder sb = new StringBuilder("● ")
                    .append(CopterMode.name(st.customMode))
                    .append(" · ").append(st.armed ? "Armed" : "Disarmed");
            if (st.hasGlobalPosition && st.armed)
                sb.append(String.format(Locale.US, " · %.0f m", st.relAlt));
            statusText.setText(sb);
            statusText.setTextColor(COLOR_OK);
        }

        // settings drawer
        connectBtn.setText(running ? R.string.disconnect : R.string.connect);
        portEdit.setEnabled(!running);
        if (!running) {
            linkDetail.setText("Not listening.");
        } else if (!connected) {
            linkDetail.setText("Listening on UDP " + s.link.getPort()
                    + ". Point the drone / SITL output at this device.");
        } else {
            InetSocketAddress a = s.link.getVehicleAddress();
            String detail = "Drone sys " + s.link.getTargetSystem() + " at "
                    + (a != null ? a.getAddress().getHostAddress() + ":" + a.getPort() : "?");
            if (st.lastStatusText != null)
                detail += "\nLast message: " + st.lastStatusText;
            linkDetail.setText(detail);
        }

        boolean flightActive = s.flight.isActive(connected, st.armed, System.currentTimeMillis());
        safety.refresh(flightActive);
        planning.refresh(flightActive);
        search.refresh(flightActive);
        demo.refresh(flightActive);
    }

    public void dispose() {
        disposed = true;
        main.removeCallbacksAndMessages(null);
        planning.dispose();
        search.dispose();
        safety.dispose();
    }
}
