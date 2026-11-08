package com.atakmap.android.plugintemplate.plugin.ui;

import android.graphics.PointF;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.atakmap.android.maps.MapEvent;
import com.atakmap.android.maps.MapEventDispatcher;
import com.atakmap.android.maps.Marker;
import com.atakmap.android.plugintemplate.plugin.R;
import com.atakmap.android.plugintemplate.plugin.mavlink.CopterMode;
import com.atakmap.android.plugintemplate.plugin.mavlink.MissionBuilder;
import com.atakmap.android.plugintemplate.plugin.mavlink.MissionUploader;
import com.atakmap.android.plugintemplate.plugin.mavlink.VehicleState;
import com.atakmap.android.plugintemplate.plugin.planning.RoutePlanner;
import com.atakmap.android.plugintemplate.plugin.planning.Waypoint;
import com.atakmap.android.plugintemplate.plugin.terrain.DtedDownloader;
import com.atakmap.coremap.maps.coords.GeoPoint;
import com.atakmap.coremap.maps.coords.GeoPointMetaData;

import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import io.dronefleet.mavlink.common.MissionItemInt;

/**
 * Mission tab, as a guided flow:
 * 1) pick destination → 2) review route (3 WPs, 100 m above terrain at each) →
 * one primary button that walks Upload → Start.
 */
public class PlanningTabController {

    /** Re-plan from the vehicle at upload if it is farther than this from the planned start. */
    private static final double REPLAN_THRESHOLD_M = 20;

    private final Services s;
    private final Handler main = new Handler(Looper.getMainLooper());
    private final ExecutorService planExec = Executors.newSingleThreadExecutor();
    private final RoutePreview preview;

    private final Button pickBtn;
    private final Button primaryBtn;
    private final View routeSection;
    private final TextView destText;
    private final TextView routeSummary;
    private final TextView wpText;
    private final TextView warnText;
    private final TextView hintText;
    private final TextView logText;

    private boolean picking;
    private volatile boolean planning;
    private double destLat = Double.NaN;
    private double destLon = Double.NaN;
    private volatile RoutePlanner.Plan plan;
    private volatile boolean uploaded;
    private boolean missionStarted;

    public PlanningTabController(View root, Services services) {
        this.s = services;
        this.preview = new RoutePreview(services.mapView);
        pickBtn = root.findViewById(R.id.pick_btn);
        primaryBtn = root.findViewById(R.id.primary_btn);
        routeSection = root.findViewById(R.id.route_section);
        destText = root.findViewById(R.id.dest_text);
        routeSummary = root.findViewById(R.id.route_summary);
        wpText = root.findViewById(R.id.wp_text);
        warnText = root.findViewById(R.id.warn_text);
        hintText = root.findViewById(R.id.hint_text);
        logText = root.findViewById(R.id.planning_log);

        pickBtn.setOnClickListener(v -> {
            if (picking)
                endPick();
            else
                beginPick();
        });
        primaryBtn.setOnClickListener(v -> {
            if (missionOnVehicle())
                confirmStart();
            else
                upload();
        });
    }

    /** This tab's current plan is the mission on the vehicle (the Search tab may have replaced it). */
    private boolean missionOnVehicle() {
        RoutePlanner.Plan p = plan;
        return uploaded && p != null && s.missionOnVehicle == p;
    }

    private void log(String line) {
        Services.appendLog(logText, line);
    }

    // ---- destination picking ---------------------------------------------

    private void beginPick() {
        final MapEventDispatcher d = s.mapView.getMapEventDispatcher();
        d.pushListeners();
        d.clearListeners(MapEvent.MAP_CLICK);
        d.clearListeners(MapEvent.ITEM_CLICK);
        MapEventDispatcher.MapEventDispatchListener l = event -> {
            PointF pt = event.getPointF();
            GeoPointMetaData gp = pt != null ? s.mapView.inverseWithElevation(pt.x, pt.y) : null;
            endPick();
            if (gp == null || gp.get() == null || !gp.get().isValid()) {
                s.toast("Could not read that map location");
                return;
            }
            onDestinationPicked(gp.get().getLatitude(), gp.get().getLongitude());
        };
        d.addMapEventListener(MapEvent.MAP_CLICK, l);
        d.addMapEventListener(MapEvent.ITEM_CLICK, l);
        picking = true;
        pickBtn.setText(R.string.pick_cancel);
        s.toast("Tap the destination on the map");
    }

    void endPick() {
        if (!picking)
            return;
        picking = false;
        s.mapView.getMapEventDispatcher().popListeners();
        updatePickLabel();
    }

    private void updatePickLabel() {
        pickBtn.setText(Double.isNaN(destLat) ? R.string.pick_on_map : R.string.pick_change);
    }

    private void onDestinationPicked(double lat, double lon) {
        destLat = lat;
        destLon = lon;
        destText.setText(String.format(Locale.US, "%.5f, %.5f", lat, lon));
        updatePickLabel();
        replan(null);
    }

    // ---- planning ----------------------------------------------------------

    /** @return {lat, lon, fromVehicle(1/0)} or null */
    private double[] currentStart() {
        VehicleState st = s.link.getState();
        if (s.link.isVehicleConnected() && st.hasGlobalPosition)
            return new double[] {st.lat, st.lon, 1};
        Marker self = s.mapView.getSelfMarker();
        if (self != null) {
            GeoPoint p = self.getPoint();
            if (p != null && p.isValid() && (p.getLatitude() != 0 || p.getLongitude() != 0))
                return new double[] {p.getLatitude(), p.getLongitude(), 0};
        }
        return null;
    }

    private void replan(Runnable then) {
        if (Double.isNaN(destLat))
            return;
        final double[] start = currentStart();
        if (start == null) {
            log("No start position — connect the drone or get a GPS fix");
            return;
        }
        uploaded = false;
        missionStarted = false;
        planning = true;
        final double dLat = destLat, dLon = destLon;
        log(start[2] == 1 ? "Planning from drone position…" : "Planning from your position…");
        planExec.execute(() -> {
            try {
                // Fetch the best terrain the DTED server has for every 1° cell the route touches.
                if (s.dtedDownloader != null) {
                    s.dtedDownloader.ensureCells(DtedDownloader.cellsFor(
                            RoutePlanner.samplePath(start[0], start[1], dLat, dLon, 2000)),
                            msg -> main.post(() -> log(msg)));
                }
                RoutePlanner.Plan p = RoutePlanner.plan(start[0], start[1], dLat, dLon,
                        s.elevation, RoutePlanner.DEFAULT_AGL_M);
                main.post(() -> {
                    planning = false;
                    plan = p;
                    showPlan(p, start[2] == 1);
                    if (then != null)
                        then.run();
                });
            } catch (RoutePlanner.NoTerrainException e) {
                main.post(() -> {
                    planning = false;
                    plan = null;
                    preview.clear();
                    routeSection.setVisibility(View.GONE);
                    log(e.getMessage());
                });
            } catch (InterruptedException e) {
                planning = false;
            }
        });
    }

    private void showPlan(RoutePlanner.Plan p, boolean fromDrone) {
        routeSection.setVisibility(View.VISIBLE);

        Set<String> sources = new LinkedHashSet<>();
        for (Waypoint wp : p.waypoints)
            if (!wp.terrainSource.isEmpty())
                sources.add(wp.terrainSource);
        routeSummary.setText(String.format(Locale.US, "%.1f km from %s · %.0f m above terrain%s",
                p.totalDistanceM / 1000.0, fromDrone ? "drone" : "you", p.aglM,
                sources.isEmpty() ? "" : " · " + String.join("/", sources)));

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < p.waypoints.size(); i++) {
            Waypoint wp = p.waypoints.get(i);
            if (sb.length() > 0)
                sb.append('\n');
            sb.append(String.format(Locale.US, "WP%d  %4.0f m MSL   ground %.0f m",
                    i + 1, wp.targetAmsl, wp.terrainMsl));
        }
        wpText.setText(sb);

        StringBuilder warn = new StringBuilder();
        if (!Double.isNaN(p.minLegClearanceM) && p.minLegClearanceM < 30) {
            warn.append(String.format(Locale.US,
                    "⚠ Terrain between waypoints comes within %.0f m of the path.",
                    p.minLegClearanceM));
        }
        for (Waypoint wp : p.waypoints) {
            if (wp.isCoarseTerrain()) {
                if (warn.length() > 0)
                    warn.append('\n');
                warn.append("⚠ Only coarse terrain (DTED0, ~900 m) here — heights are approximate.");
                break;
            }
        }
        warnText.setVisibility(warn.length() > 0 ? View.VISIBLE : View.GONE);
        warnText.setText(warn);
        preview.show(p);
    }

    // ---- upload / start ---------------------------------------------------

    private void upload() {
        if (plan == null || !s.link.isVehicleConnected())
            return;
        VehicleState st = s.link.getState();
        if (st.hasGlobalPosition && RoutePlanner.distance(plan.startLat, plan.startLon,
                st.lat, st.lon) > REPLAN_THRESHOLD_M) {
            log("Re-planning from the drone's position");
            replan(this::doUpload);
        } else {
            doUpload();
        }
    }

    private void doUpload() {
        final RoutePlanner.Plan p = plan;
        if (p == null)
            return;
        s.runner.start("Upload", this::log, ctx -> {
            VehicleState st = s.link.getState();
            double homeLat = st.hasGlobalPosition ? st.lat : p.startLat;
            double homeLon = st.hasGlobalPosition ? st.lon : p.startLon;
            double homeAlt = st.hasGlobalPosition ? st.altAmsl
                    : (Double.isNaN(p.startTerrainMsl) ? 0 : p.startTerrainMsl);
            List<MissionItemInt> items = MissionBuilder.build(
                    s.link.getTargetSystem(), s.link.getTargetComponent(),
                    homeLat, homeLon, homeAlt, (float) p.aglM, p.waypoints);
            ctx.log("Uploading " + items.size() + " mission items…");
            new MissionUploader(s.link).upload(items, null);
            s.missionOnVehicle = p;
            uploaded = true;
        }, null);
    }

    private void confirmStart() {
        if (!missionOnVehicle())
            return;
        s.confirm("Start mission?",
                String.format(Locale.US, "The drone will arm, take off to %.0f m, fly the "
                        + "3 waypoints, then return home.\n\nIs the area clear?",
                        plan != null ? plan.aglM : RoutePlanner.DEFAULT_AGL_M),
                this::startMission);
    }

    private void startMission() {
        s.flight.started(System.currentTimeMillis());
        missionStarted = true;
        s.runner.start("Start", this::log, ctx -> {
            VehicleState st = s.link.getState();
            if (!st.armed) {
                ctx.log("GUIDED → arming");
                s.commands.setModeBlocking(CopterMode.GUIDED);
                s.commands.armBlocking(true);
            }
            ctx.log("AUTO → mission start");
            s.commands.setModeBlocking(CopterMode.AUTO);
            s.commands.missionStartBlocking();
        }, null);
    }

    // ---- state → UI --------------------------------------------------------

    /** Called by the UI ticker. */
    void refresh(boolean flightActive) {
        boolean connected = s.link.isVehicleConnected();
        boolean busy = s.runner.isBusy();
        if (missionStarted && !flightActive && !busy)
            missionStarted = false; // flight finished: allow flying the same mission again

        String label;
        String hint;
        boolean enabled;
        if (picking) {
            label = "Pick a destination";
            hint = "Tap a point on the map";
            enabled = false;
        } else if (planning) {
            label = "Planning route…";
            hint = "Fetching terrain and computing waypoints";
            enabled = false;
        } else if (plan == null) {
            label = "Pick a destination";
            hint = "";
            enabled = false;
        } else if (busy) {
            label = uploaded ? "Starting…" : "Uploading…";
            hint = "";
            enabled = false;
        } else if (missionStarted) {
            label = "Mission in progress";
            hint = "Use R2Home, Land or KILL above";
            enabled = false;
        } else if (!missionOnVehicle()) {
            label = "Upload mission";
            hint = connected ? "Sends takeoff, 3 waypoints and return-home to the drone"
                    : "Connect a drone first (⚙)";
            enabled = connected;
        } else {
            label = "Start mission";
            hint = connected ? "Mission uploaded ✓" : "Drone disconnected";
            enabled = connected;
        }
        // nothing to act on until a route exists: let the pick button lead
        boolean showPrimary = planning || plan != null;
        primaryBtn.setVisibility(showPrimary ? View.VISIBLE : View.GONE);
        hintText.setVisibility(showPrimary ? View.VISIBLE : View.GONE);
        primaryBtn.setText(label);
        primaryBtn.setEnabled(enabled);
        primaryBtn.setAlpha(enabled ? 1f : 0.45f);
        hintText.setText(hint);
        pickBtn.setEnabled(!busy && !missionStarted);
    }

    void dispose() {
        endPick();
        preview.clear();
        planExec.shutdownNow();
    }
}
