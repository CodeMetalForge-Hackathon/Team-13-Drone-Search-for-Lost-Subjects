package com.atakmap.android.plugintemplate.plugin.ui;

import android.app.AlertDialog;
import android.content.Context;
import android.widget.TextView;
import android.widget.Toast;

import com.atakmap.android.maps.MapView;
import com.atakmap.android.plugintemplate.plugin.mavlink.MavlinkLink;
import com.atakmap.android.plugintemplate.plugin.mavlink.VehicleCommands;
import com.atakmap.android.plugintemplate.plugin.terrain.DtedDownloader;
import com.atakmap.android.plugintemplate.plugin.terrain.ElevationSource;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/** Shared plumbing handed to every UI controller. */
public class Services {

    public final Context pluginContext;
    public final MapView mapView;
    public final MavlinkLink link;
    public final VehicleCommands commands;
    public final SequenceRunner runner;
    public final ElevationSource elevation;
    public final DtedDownloader dtedDownloader;
    public final FlightSession flight = new FlightSession();
    /**
     * The plan whose mission is on the vehicle now (set after a successful upload by the Plan or the
     * Search tab), so one tab never starts a mission the other tab uploaded.
     */
    public volatile Object missionOnVehicle;

    public Services(Context pluginContext, MapView mapView, MavlinkLink link,
            VehicleCommands commands, SequenceRunner runner, ElevationSource elevation,
            DtedDownloader dtedDownloader) {
        this.pluginContext = pluginContext;
        this.mapView = mapView;
        this.link = link;
        this.commands = commands;
        this.runner = runner;
        this.elevation = elevation;
        this.dtedDownloader = dtedDownloader;
    }

    /** Activity context for dialogs/toasts (plugin context cannot host windows). */
    public Context hostContext() {
        return mapView != null ? mapView.getContext() : pluginContext;
    }

    public void toast(String msg) {
        Toast.makeText(hostContext(), msg, Toast.LENGTH_LONG).show();
    }

    public void confirm(String title, String message, Runnable onYes) {
        new AlertDialog.Builder(hostContext())
                .setTitle(title)
                .setMessage(message)
                .setPositiveButton("Proceed", (d, w) -> onYes.run())
                .setNegativeButton("Cancel", null)
                .show();
    }

    private static final int MAX_LOG_LINES = 5;

    /** Keeps only the last few lines so the pane stays readable. */
    public static void appendLog(TextView view, String line) {
        String ts = new SimpleDateFormat("HH:mm:ss", Locale.US).format(new Date());
        CharSequence old = view.getText();
        String text = (old.length() == 0 ? "" : old + "\n") + ts + "  " + line;
        String[] lines = text.split("\n");
        if (lines.length > MAX_LOG_LINES) {
            StringBuilder sb = new StringBuilder();
            for (int i = lines.length - MAX_LOG_LINES; i < lines.length; i++) {
                if (sb.length() > 0)
                    sb.append('\n');
                sb.append(lines[i]);
            }
            text = sb.toString();
        }
        view.setText(text);
    }
}
