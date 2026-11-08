package com.atakmap.android.plugintemplate.plugin.ui;

import android.os.Handler;
import android.text.Editable;
import android.text.TextWatcher;
import android.os.Looper;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;

import com.atakmap.android.plugintemplate.plugin.R;
import com.atakmap.android.plugintemplate.plugin.mavlink.CopterMode;
import com.atakmap.android.plugintemplate.plugin.mavlink.MissionUploader;
import com.atakmap.android.plugintemplate.plugin.mavlink.SearchMissionBuilder;
import com.atakmap.android.plugintemplate.plugin.mavlink.VehicleState;
import com.atakmap.android.plugintemplate.plugin.planning.RoutePlanner;
import com.atakmap.android.plugintemplate.plugin.search.ArduCopterTiming;
import com.atakmap.android.plugintemplate.plugin.search.AtakMapAccess;
import com.atakmap.android.plugintemplate.plugin.search.BitmapTileDecoder;
import com.atakmap.android.plugintemplate.plugin.search.DroneTypes;
import com.atakmap.android.plugintemplate.plugin.search.SearchOverlay;
import com.atakmap.android.plugintemplate.plugin.search.TerrainElevation;
import com.atakmap.android.plugintemplate.plugin.terrain.DtedDownloader;
import com.mindfulpigeon.atak.bridge.DroneProfile;
import com.mindfulpigeon.atak.bridge.MissionForm;
import com.mindfulpigeon.atak.bridge.PluginConfig;
import com.mindfulpigeon.atak.bridge.PluginController;
import com.mindfulpigeon.atak.bridge.PluginPorts;
import com.mindfulpigeon.atak.bridge.TakMapAccess;
import com.mindfulpigeon.search.api.BeliefSummary;
import com.mindfulpigeon.search.api.DroneRoute;
import com.mindfulpigeon.search.api.SearchPlan;
import com.mindfulpigeon.search.api.SearchSession;
import com.mindfulpigeon.search.api.TargetBehavior;
import com.mindfulpigeon.search.api.Waypoint;
import com.mindfulpigeon.search.geo.LatLon;
import com.mindfulpigeon.search.prior.SereInputs;

import com.atakmap.coremap.maps.coords.GeoPoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeSet;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Search tab: plans drone search routes (the search engine under search/) over the area the
 * operator draws on the map or picks from the TAK shapes, with the probability raised around any
 * places the operator marks as where the person may be. The last known position, suspected
 * locations and red zones are placed with a tap or by MGRS grid ({@link SearchMapTools}). By default the route searches the most likely areas first, then the rest of the
 * area while flight time lasts (probability-ordered coverage). Each drone has a type (its speed,
 * turns, launch, recovery and time in the air, {@link DroneTypes}); the operator sets how long the
 * area is searched, and a drone that lands before that continues with Next flight. Uploads
 * drone-1's route to the MAVLink drone (ArduCopter) with terrain-safe altitudes, starts it, and
 * feeds the drone's telemetry back into the in-flight belief so Re-plan can redirect it; other
 * types get a plan to fly from their own ground station.
 *
 * <p>All planning logic lives in the SDK-free {@link PluginController}; this class binds widgets,
 * the drone link, terrain downloads and the map.
 */
public class SearchTabController {

    /** The MAVLink vehicle flies the plan's first route. */
    static final String DRONE_ID = "drone-1";
    private static final String TAG = "KoalifiedSearch";
    private static final long REPORT_MS = 1000;
    /** Warn when the drone is this far from the plan's launch point at upload. */
    private static final double LAUNCH_WARN_M = 50;
    /** Spacing of the points used to find the 1° terrain cells a search area covers. */
    private static final double AREA_SAMPLE_DEG = 0.25;
    /** Search time slider: 5 min steps, 5 min to 8 h. */
    private static final int SEARCH_STEP_MIN = 5;
    /** Below this much search time left, Next flight does not plan another flight. */
    private static final double MIN_NEXT_FLIGHT_S = 120;

    private final Services s;
    private final Handler main = new Handler(Looper.getMainLooper());
    private final ExecutorService terrainExec = daemon("KoalifiedSearchTerrain");
    private final ExecutorService telemetryExec = daemon("KoalifiedSearchTelemetry");
    private final PluginConfig cfg = new PluginConfig();
    private final MissionForm form = new MissionForm();
    private final SearchOverlay overlay;
    private final AtakMapAccess map;
    private final PluginController controller;

    private final EditText minutesEdit;
    private final EditText lkpRadiusEdit;
    private final EditText dronesEdit;
    private final Button[] droneTypeBtns = new Button[3];
    private final ChoiceRow<SereInputs.Environment> environmentRow;
    private final ChoiceRow<SereInputs.Mobility> mobilityRow;
    private final ChoiceRow<TargetBehavior> behaviorRow;
    private final ChoiceRow<String> patternRow;
    private final TextView patternDesc;
    private final SearchMapTools tools;
    private final TextView areaText;
    private final Button drawAreaBtn;
    private final Button pickAreaBtn;
    private final ViewGroup drawPanel;
    private final ViewGroup drawSlotArea;
    private final ViewGroup drawSlotRedZone;
    private final TextView drawStatus;
    private final Button drawUndoBtn;
    private final Button drawDoneBtn;
    private final TextView lkpText;
    private final Button placeLkpBtn;
    private final TextView suspectsText;
    private final Button addSuspectBtn;
    private final Button clearSuspectsBtn;
    private final TextView redZonesText;
    private final Button drawRedZoneBtn;
    private final TextView launchText;
    private final Button placeLaunchBtn;
    private final Button resetBtn;
    private final TextView flightLabel;
    private final Button planBtn;
    private final Button cancelBtn;
    private final ProgressBar progress;
    private final TextView statusText;
    private final TextView routesText;
    private final TextView warnText;
    private final Button uploadBtn;
    private final Button startBtn;
    private final TextView hintText;
    private final Button replanBtn;
    private final Button nextFlightBtn;
    private final Button endBtn;
    private final TextView beliefText;
    private final TextView logText;

    // UI thread state
    private boolean planning;
    private boolean fetchingTerrain;
    private SearchPlan fullPlan; // last full (non-preview) plan or re-plan
    private boolean missionRunning;
    private long lastReportMs;
    private long lastTimerRefreshMs;
    /** Set by the upload task once drone-1's route of this plan is on the vehicle. */
    private volatile SearchPlan uploadedPlan;
    /** A telemetry update is still running: skip new ones instead of queueing them. */
    private final AtomicBoolean telemetryBusy = new AtomicBoolean();

    /**
     * The search patterns: the planner name, the option's label, and what the route looks like.
     * They differ in the shape of the route as much as in what they aim for.
     */
    private static final String[][] PATTERNS = {
            {"POC", "Straight sweeps",
                    "Hotspots first: straight back-and-forth passes over the most likely areas, "
                            + "then over the rest while flight time lasts. Predictable to fly "
                            + "and to watch; covers the area in order."},
            {"auto", "Free route",
                    "Fastest find: a winding route that follows the probability cell by cell "
                            + "for the best chance of an early find. Can leave unlikely parts "
                            + "of the area unsearched."}};
    /** Map-read texts (LKP, red zones) refresh this often while the tab runs. */
    private static final long MAP_TEXT_REFRESH_MS = 2000;

    /** Flights flown in this search (1 after Start search); a next flight's plan until it starts. */
    private int flightNumber;
    private boolean awaitingNextFlight;
    private SearchPlan nextFlightPlan;

    /** Each drone's type, an index into DroneTypes.ALL (launch order). */
    private final int[] droneType = {0, 0, 0};
    /** How long the operator wants the area searched, s: the drones' time in the air, all flights. */
    private double searchTimeS;
    /** Seconds the drone has been in the air in this search. */
    private double airborneS;
    private long lastAirborneTickMs;
    private int shownAirborneMin = -1;
    /** Drone-1's type in the plan last requested, and in the plan shown (what an upload would fly). */
    private DroneProfile requestedDrone1;
    private DroneProfile planDrone1;

    private String areaTitle;
    private final Map<String, String> suspectTitles = new LinkedHashMap<>();
    private long lastMapTextMs;

    private final PluginPorts.Ui ui = new PluginPorts.Ui() {
        @Override
        public void planning(boolean running, boolean preview) {
            planning = running && !preview;
            progress.setVisibility(planning ? View.VISIBLE : View.GONE);
            if (planning) {
                progress.setProgress(0);
                statusText.setText("Planning…");
            }
        }

        @Override
        public void progress(String stage, double fraction, double best) {
            statusText.setText(String.format(Locale.US, "Planning: %s · chance of finding %.0f%%",
                    stage, 100 * best));
            if (fraction >= 0)
                progress.setProgress((int) (fraction * 100));
        }

        @Override
        public void planned(SearchPlan plan, boolean preview) {
            showPlan(plan, preview);
        }

        @Override
        public void belief(BeliefSummary b) {
            beliefText.setText(String.format(Locale.US,
                    "Searched %.0f%% of the area · chance found by now %.0f%% · still in the area %.0f%%%s",
                    100 * b.coveredFraction, 100 * b.cumulativePos, 100 * b.probabilityInArea,
                    b.movingTarget ? String.format(Locale.US, " · hiding %.0f%%",
                            100 * b.probabilityHidden) : ""));
        }

        @Override
        public void error(String message) {
            planning = false;
            awaitingNextFlight = false;
            progress.setVisibility(View.GONE);
            statusText.setText("⚠ " + message);
        }
    };

    public SearchTabController(View root, Services services) {
        this.s = services;
        // Waypoint heights stay above ground (AGL); SearchMissionBuilder turns them into
        // terrain-safe MSL altitudes from the plugin's DTED at upload.
        cfg.terrainFollowing = false;
        // plan with ArduCopter's real speed, turn and takeoff/landing times (fitted in SITL)
        ArduCopterTiming.apply(cfg);

        minutesEdit = root.findViewById(R.id.search_minutes);
        lkpRadiusEdit = root.findViewById(R.id.search_lkp_radius);
        dronesEdit = root.findViewById(R.id.search_drones);
        droneTypeBtns[0] = root.findViewById(R.id.search_drone_type_1);
        droneTypeBtns[1] = root.findViewById(R.id.search_drone_type_2);
        droneTypeBtns[2] = root.findViewById(R.id.search_drone_type_3);
        patternDesc = root.findViewById(R.id.search_pattern_desc);
        areaText = root.findViewById(R.id.search_area_text);
        drawAreaBtn = root.findViewById(R.id.search_draw_area_btn);
        pickAreaBtn = root.findViewById(R.id.search_pick_area_btn);
        drawPanel = root.findViewById(R.id.search_draw_panel);
        drawSlotArea = root.findViewById(R.id.search_draw_slot_area);
        drawSlotRedZone = root.findViewById(R.id.search_draw_slot_red_zone);
        drawStatus = root.findViewById(R.id.search_draw_status);
        drawUndoBtn = root.findViewById(R.id.search_draw_undo_btn);
        drawDoneBtn = root.findViewById(R.id.search_draw_done_btn);
        lkpText = root.findViewById(R.id.search_lkp_text);
        placeLkpBtn = root.findViewById(R.id.search_place_lkp_btn);
        suspectsText = root.findViewById(R.id.search_suspects_text);
        addSuspectBtn = root.findViewById(R.id.search_add_suspect_btn);
        clearSuspectsBtn = root.findViewById(R.id.search_clear_suspects_btn);
        redZonesText = root.findViewById(R.id.search_red_zones_text);
        drawRedZoneBtn = root.findViewById(R.id.search_draw_red_zone_btn);
        launchText = root.findViewById(R.id.search_launch_text);
        placeLaunchBtn = root.findViewById(R.id.search_place_launch_btn);
        flightLabel = root.findViewById(R.id.search_flight_label);
        planBtn = root.findViewById(R.id.search_plan_btn);
        cancelBtn = root.findViewById(R.id.search_cancel_btn);
        progress = root.findViewById(R.id.search_progress);
        statusText = root.findViewById(R.id.search_status);
        routesText = root.findViewById(R.id.search_routes);
        warnText = root.findViewById(R.id.search_warn);
        uploadBtn = root.findViewById(R.id.search_upload_btn);
        startBtn = root.findViewById(R.id.search_start_btn);
        hintText = root.findViewById(R.id.search_hint_text);
        replanBtn = root.findViewById(R.id.search_replan_btn);
        nextFlightBtn = root.findViewById(R.id.search_next_flight_btn);
        endBtn = root.findViewById(R.id.search_end_btn);
        beliefText = root.findViewById(R.id.search_belief);
        logText = root.findViewById(R.id.search_log);

        overlay = new SearchOverlay(s.mapView, s::toast);
        map = new AtakMapAccess(s.mapView, this::dronePosition, cfg.launchPrefix,
                SearchOverlay.GROUP_PLAN, SearchOverlay.GROUP_BELIEF, RoutePreview.GROUP_NAME,
                SearchMapTools.GROUP_DRAFT);
        controller = new PluginController(map, new TerrainElevation(s.elevation),
                new BitmapTileDecoder(), overlay, ui, main::post, cfg);
        tools = new SearchMapTools(s, cfg, map, new SearchMapTools.Listener() {
            @Override
            public void searchArea(String uid, String title) {
                form.searchAreaUid = uid;
                areaTitle = title;
                updateMapInputs();
            }

            @Override
            public void suspect(String uid, String title) {
                if (!form.suspectUids.contains(uid)) {
                    form.suspectUids.add(uid);
                    suspectTitles.put(uid, title);
                }
                updateMapInputs();
            }

            @Override
            public void changed() {
                updateMapInputs();
            }
        });

        // the operator's defaults: an evader in hostile ground, uninjured and hiding; the route
        // searches the most likely areas first, then the rest while flight time lasts
        form.environment = SereInputs.Environment.HOSTILE;
        form.mobility = SereInputs.Mobility.UNINJURED;
        form.targetBehavior = TargetBehavior.CONCEALING;
        form.planner = PATTERNS[0][0];
        environmentRow = new ChoiceRow<>(root.findViewById(R.id.search_environment_row),
                Arrays.asList(SereInputs.Environment.values()),
                labels(SereInputs.Environment.values()), form.environment,
                v -> form.environment = v);
        mobilityRow = new ChoiceRow<>(root.findViewById(R.id.search_mobility_row),
                Arrays.asList(SereInputs.Mobility.values()),
                labels(SereInputs.Mobility.values()), form.mobility, v -> form.mobility = v);
        behaviorRow = new ChoiceRow<>(root.findViewById(R.id.search_behavior_row),
                Arrays.asList(TargetBehavior.values()), labels(TargetBehavior.values()),
                form.targetBehavior, v -> form.targetBehavior = v);
        List<String> plannerNames = new ArrayList<>();
        List<String> patternLabels = new ArrayList<>();
        for (String[] p : PATTERNS) {
            plannerNames.add(p[0]);
            patternLabels.add(p[1]);
        }
        patternRow = new ChoiceRow<>(root.findViewById(R.id.search_pattern_row), plannerNames,
                patternLabels, form.planner, v -> {
                    form.planner = v;
                    updatePatternText();
                });
        updatePatternText();

        drawAreaBtn.setOnClickListener(v -> tools.toggle(SearchMapTools.Mode.DRAW_AREA));
        pickAreaBtn.setOnClickListener(v -> tools.toggle(SearchMapTools.Mode.PICK_AREA));
        drawRedZoneBtn.setOnClickListener(v -> tools.toggle(SearchMapTools.Mode.DRAW_RED_ZONE));
        placeLkpBtn.setOnClickListener(v -> tools.toggle(SearchMapTools.Mode.PLACE_LKP));
        addSuspectBtn.setOnClickListener(v -> tools.toggle(SearchMapTools.Mode.PLACE_SUSPECT));
        root.findViewById(R.id.search_lkp_mgrs_btn).setOnClickListener(v -> tools.enterLkpMgrs());
        root.findViewById(R.id.search_suspect_mgrs_btn)
                .setOnClickListener(v -> tools.enterSuspectMgrs());
        root.findViewById(R.id.search_red_zone_mgrs_btn)
                .setOnClickListener(v -> tools.enterRedZoneMgrs());
        drawUndoBtn.setOnClickListener(v -> tools.undoCorner());
        drawDoneBtn.setOnClickListener(v -> tools.finishShape());
        root.findViewById(R.id.search_draw_cancel_btn).setOnClickListener(v -> tools.stop());
        clearSuspectsBtn.setOnClickListener(v -> {
            suspectTitles.clear();
            form.suspectUids.clear();
            updateMapInputs();
        });
        root.findViewById(R.id.search_launch_here_btn)
                .setOnClickListener(v -> tools.launchAtMyLocation());
        placeLaunchBtn.setOnClickListener(v -> tools.toggle(SearchMapTools.Mode.PLACE_LAUNCH));
        root.findViewById(R.id.search_launch_mgrs_btn)
                .setOnClickListener(v -> tools.enterLaunchMgrs());
        resetBtn = root.findViewById(R.id.search_reset_btn);
        resetBtn.setOnClickListener(v -> confirmReset());
        updateMapInputs();
        for (int i = 0; i < droneTypeBtns.length; i++) {
            final int k = i;
            droneTypeBtns[i].setOnClickListener(v -> {
                droneType[k] = (droneType[k] + 1) % DroneTypes.ALL.size();
                updateDroneTypes();
            });
        }
        dronesEdit.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence t, int start, int count, int after) {
            }

            @Override
            public void onTextChanged(CharSequence t, int start, int before, int count) {
            }

            @Override
            public void afterTextChanged(Editable e) {
                updateDroneTypes();
            }
        });

        SeekBar flight = root.findViewById(R.id.search_flight_time);
        setSearchTime(flight.getProgress());
        updateDroneTypes();
        flight.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar bar, int step, boolean fromUser) {
                setSearchTime(step);
            }

            @Override
            public void onStartTrackingTouch(SeekBar bar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar bar) {
                if (missionRunning)
                    return; // in flight, route time comes from the drones' remaining routes
                readFields();
                controller.preview(form); // quick preview, dashed on the map
            }
        });

        planBtn.setOnClickListener(v -> plan());
        cancelBtn.setOnClickListener(v -> controller.cancel());
        uploadBtn.setOnClickListener(v -> upload());
        startBtn.setOnClickListener(v -> confirmStart());
        replanBtn.setOnClickListener(v -> controller.replan(System.currentTimeMillis()));
        nextFlightBtn.setOnClickListener(v -> nextFlight());
        endBtn.setOnClickListener(v -> endMission());
    }

    private static ExecutorService daemon(String name) {
        return Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, name);
            t.setDaemon(true);
            return t;
        });
    }

    private void log(String line) {
        Services.appendLog(logText, line);
    }

    // ---- inputs ------------------------------------------------------------

    /** "Hostile" for HOSTILE: an option as the operator reads it. */
    private static String pretty(Enum<?> e) {
        String n = e.name().toLowerCase(Locale.US);
        return Character.toUpperCase(n.charAt(0)) + n.substring(1);
    }

    private static List<String> labels(Enum<?>[] values) {
        List<String> out = new ArrayList<>(values.length);
        for (Enum<?> v : values)
            out.add(pretty(v));
        return out;
    }

    private void updatePatternText() {
        for (String[] p : PATTERNS)
            if (p[0].equals(form.planner))
                patternDesc.setText(p[2]);
    }

    // ---- the search area, LKP, suspected locations and red zones on the map --------

    private void updateMapInputs() {
        updateMapInputs(false);
    }

    /**
     * What the tab shows about the map inputs and the map tool that is on. The LKP and red zones
     * are read from the map, where the operator may also move or delete them in ATAK; with
     * fromMap, suspected locations whose marker is gone are dropped too.
     */
    private void updateMapInputs(boolean fromMap) {
        lastMapTextMs = System.currentTimeMillis();
        SearchMapTools.Mode mode = tools.mode();
        List<TakMapAccess.Shape> shapes = map.shapes();
        List<TakMapAccess.Marker> markers = map.markers();

        boolean areaOnMap = false;
        String titledArea = null; // with none picked, the plan uses the first shape titled SEARCH
        int titledAreas = 0;
        for (TakMapAccess.Shape sh : shapes) {
            areaOnMap |= sh.uid.equals(form.searchAreaUid);
            if (SearchMapTools.titled(sh.title, cfg.searchAreaPrefix) && sh.points.size() >= 3) {
                if (titledArea == null)
                    titledArea = sh.title.trim();
                titledAreas++;
            }
        }
        if (areaTitle != null)
            areaText.setText("Search area: " + areaTitle
                    + (areaOnMap ? "" : " (no longer on the map: draw or pick it again)"));
        else if (titledArea != null)
            areaText.setText("Search area: " + titledArea + " (titled " + cfg.searchAreaPrefix
                    + (titledAreas > 1 ? ", first of " + titledAreas : "") + ")");
        else
            areaText.setText(R.string.search_area_none);
        drawAreaBtn.setText(mode == SearchMapTools.Mode.DRAW_AREA ? R.string.search_drawing
                : R.string.search_draw_area);
        pickAreaBtn.setText(mode == SearchMapTools.Mode.PICK_AREA
                ? R.string.search_pick_area_cancel : R.string.search_pick_area);
        placeLkpBtn.setText(mode == SearchMapTools.Mode.PLACE_LKP
                ? R.string.search_tap_map_cancel : R.string.search_place_lkp);
        addSuspectBtn.setText(mode == SearchMapTools.Mode.PLACE_SUSPECT
                ? R.string.search_tap_map_cancel : R.string.search_add_suspect);
        drawRedZoneBtn.setText(mode == SearchMapTools.Mode.DRAW_RED_ZONE ? R.string.search_drawing
                : R.string.search_draw_red_zone);
        placeLaunchBtn.setText(mode == SearchMapTools.Mode.PLACE_LAUNCH
                ? R.string.search_tap_map_cancel : R.string.search_place_on_map);
        updateDrawPanel(mode);
        launchText.setText(launchDescription(markers));

        // last known position: the first marker titled LKP is the one the plan reads
        TakMapAccess.Marker lkp = null;
        int lkps = 0;
        java.util.Set<String> markerUids = new java.util.HashSet<>();
        for (TakMapAccess.Marker m : markers) {
            markerUids.add(m.uid);
            if (SearchMapTools.titled(m.title, cfg.lkpPrefix)) {
                if (lkp == null)
                    lkp = m;
                lkps++;
            }
        }
        lkpText.setText(lkp == null ? getString(R.string.search_lkp_none)
                : "Last known position (LKP): "
                        + SearchMapTools.mgrs(new GeoPoint(lkp.point.lat, lkp.point.lon))
                        + (lkps > 1 ? String.format(Locale.US,
                                " · %d markers titled %s: the plan uses this one", lkps,
                                cfg.lkpPrefix) : ""));

        if (fromMap) {
            for (String uid : new ArrayList<>(form.suspectUids)) {
                if (!markerUids.contains(uid)) {
                    form.suspectUids.remove(uid);
                    suspectTitles.remove(uid);
                }
            }
        }
        suspectsText.setText(suspectTitles.isEmpty() ? getString(R.string.search_suspects_none)
                : String.format(Locale.US, "Suspected locations: %s (probability up to %.0f× within %.0f m)",
                        android.text.TextUtils.join(", ", suspectTitles.values()),
                        1 + cfg.suspectStrength, cfg.suspectRadiusM));
        setEnabled(clearSuspectsBtn, !suspectTitles.isEmpty());

        List<String> zones = new ArrayList<>();
        for (TakMapAccess.Shape sh : shapes)
            if (!sh.uid.equals(form.searchAreaUid)
                    && SearchMapTools.titled(sh.title, cfg.redZonePrefix))
                zones.add(sh.title.trim());
        redZonesText.setText(zones.isEmpty() ? getString(R.string.search_red_zones_none)
                : String.format(Locale.US, "Red zones (kept out, plus %.0f m): %s",
                        cfg.redZoneBufferM, android.text.TextUtils.join(", ", zones)));
    }

    /**
     * Where drone-1 takes off, as the plan will read it: the first marker titled LAUNCH, else the
     * connected drone's position, else the device's location.
     */
    private String launchDescription(List<TakMapAccess.Marker> markers) {
        TakMapAccess.Marker marker = null;
        boolean drone = false;
        int launches = 0;
        for (TakMapAccess.Marker m : markers) {
            if (AtakMapAccess.DRONE_LAUNCH_UID.equals(m.uid)) {
                drone = true;
            } else if (SearchMapTools.titled(m.title, cfg.launchPrefix)) {
                launches++;
                if (marker == null)
                    marker = m;
            }
        }
        int n = droneCount();
        if (marker != null) {
            // drones beyond the launch markers take off from the last one (by title)
            String shared = n <= 1 || launches >= n ? ""
                    : launches == 1 ? " · all " + n + " drones launch here"
                    : String.format(Locale.US, " · drones %d-%d launch from the last %s marker",
                            launches + 1, n, cfg.launchPrefix);
            return "Launch point: " + marker.title.trim() + " at "
                    + SearchMapTools.mgrs(new GeoPoint(marker.point.lat, marker.point.lon)) + shared;
        }
        if (drone)
            return "Launch point: where the drone is now";
        LatLon self = map.self();
        if (self != null)
            return "Launch point: your location now, "
                    + SearchMapTools.mgrs(new GeoPoint(self.lat, self.lon));
        return "Launch point: none yet (no location): set one below";
    }

    /** Asks, then removes from the map what this tab placed and clears the tab's picks. */
    private void confirmReset() {
        tools.stop();
        List<com.atakmap.android.maps.MapItem> items = tools.placedItems();
        boolean picks = form.searchAreaUid != null || !form.suspectUids.isEmpty();
        if (items.isEmpty() && !picks) {
            s.toast("Nothing to reset: this tab has placed nothing on the map");
            return;
        }
        List<String> names = new ArrayList<>();
        for (com.atakmap.android.maps.MapItem i : items)
            names.add(SearchMapTools.title(i));
        String msg = (items.isEmpty() ? "Nothing placed from this tab is on the map."
                : "Removes from the map what this tab placed: "
                        + android.text.TextUtils.join(", ", names) + ".")
                + "\n\nAlso clears the picked search area, the suspected locations and the plan "
                + "shown. Shapes and markers made elsewhere stay.";
        s.confirm("Reset the search inputs?", msg, () -> {
            if (missionRunning)
                return; // a search started while the dialog was open
            int n = tools.removePlaced();
            form.searchAreaUid = null;
            areaTitle = null;
            suspectTitles.clear();
            form.suspectUids.clear();
            // the plan shown was made from what is gone
            fullPlan = null;
            nextFlightPlan = null;
            overlay.clearAll();
            statusText.setText("");
            routesText.setText("");
            warnText.setVisibility(View.GONE);
            updateMapInputs();
            s.toast(n == 0 ? "Search inputs cleared" : "Removed " + n + " item" + (n == 1 ? "" : "s")
                    + " from the map");
        });
    }

    /** The drawing bar: under the buttons of what is being drawn, with the corners so far. */
    private void updateDrawPanel(SearchMapTools.Mode mode) {
        if (!mode.drawing()) {
            drawPanel.setVisibility(View.GONE);
            return;
        }
        boolean area = mode == SearchMapTools.Mode.DRAW_AREA;
        ViewGroup slot = area ? drawSlotArea : drawSlotRedZone;
        boolean appearing = drawPanel.getVisibility() != View.VISIBLE;
        if (drawPanel.getParent() != slot) {
            ((ViewGroup) drawPanel.getParent()).removeView(drawPanel);
            slot.addView(drawPanel);
            appearing = true;
        }
        if (appearing) // scroll the pane so Undo, Done and Cancel are in view
            drawPanel.post(() -> drawPanel.requestRectangleOnScreen(
                    new android.graphics.Rect(0, 0, drawPanel.getWidth(),
                            drawPanel.getHeight()), false));
        drawPanel.setBackgroundColor(area ? 0x33FFEB3B : 0x44E53935);
        int n = tools.corners();
        String what = area ? "Search area" : "Red zone";
        drawStatus.setText(n < ShapeDraft.MIN_CORNERS
                ? String.format(Locale.US, "%s: tap the map at each corner (%d of at least %d)",
                        what, n, ShapeDraft.MIN_CORNERS)
                : String.format(Locale.US, "%s: %d corners. Tap to add more, or Done", what, n));
        setEnabled(drawUndoBtn, n > 0);
        setEnabled(drawDoneBtn, tools.canFinish());
        drawPanel.setVisibility(View.VISIBLE);
    }

    private String getString(int id) {
        return s.pluginContext.getString(id);
    }

    private void setSearchTime(int step) {
        searchTimeS = SEARCH_STEP_MIN * 60.0 * (Math.max(0, step) + 1);
        updateSearchTimeLabel();
    }

    private static String hoursMinutes(double seconds) {
        long m = Math.round(seconds / 60);
        return m < 60 ? m + " min" : String.format(Locale.US, "%d h %02d min", m / 60, m % 60);
    }

    /** Drones in the form (1-3), from the field as typed. */
    private int droneCount() {
        return (int) Math.max(1, Math.min(3, parse(dronesEdit, form.droneCount)));
    }

    private DroneProfile typeOf(int i) {
        return DroneTypes.ALL.get(droneType[i]);
    }

    /** The search time, how many flights each drone needs for it, and what has been flown. */
    private void updateSearchTimeLabel() {
        StringBuilder sb = new StringBuilder("Search time: ").append(hoursMinutes(searchTimeS));
        int n = droneCount();
        for (int i = 0; i < n; i++) {
            DroneProfile p = typeOf(i);
            int flights = (int) Math.ceil(searchTimeS / p.enduranceS - 1e-9);
            if (flights > 1)
                sb.append(String.format(Locale.US, " · drone %d: %d flights (%s each)", i + 1,
                        flights, hoursMinutes(p.enduranceS)));
        }
        if (missionRunning)
            sb.append(String.format(Locale.US, " · %s flown", hoursMinutes(airborneS)));
        else
            sb.append(" (release to preview)");
        flightLabel.setText(sb);
    }

    private void updateDroneTypes() {
        int n = droneCount();
        for (int i = 0; i < droneTypeBtns.length; i++) {
            Button b = droneTypeBtns[i];
            b.setVisibility(i < n ? View.VISIBLE : View.GONE);
            DroneProfile p = typeOf(i);
            String camera = !p.hasCamera() ? ""
                    : String.format(Locale.US, " · sees %.0f m per pass (%d cells) from %.0f ft",
                            p.swathM, Math.round(p.laneSpacingM(cfg.cellSizeM) / cfg.cellSizeM),
                            p.searchAltitudeAglM / 0.3048);
            b.setText(String.format(Locale.US, "Drone %d: %s · %s · %s in the air%s%s  ▸", i + 1,
                    p.name, p.kind, hoursMinutes(p.enduranceS), camera,
                    i == 0 && !p.mavlinkUpload ? " · plan only" : ""));
            // a next flight continues the same drones' search: types stay as flown
            setEnabled(b, !missionRunning);
        }
        setEnabled(dronesEdit, !missionRunning);
        updateSearchTimeLabel();
    }

    private void readFields() {
        form.minutesSinceIncident = parse(minutesEdit, form.minutesSinceIncident);
        form.lkpRadiusM = parse(lkpRadiusEdit, form.lkpRadiusM);
        form.droneCount = droneCount();
        form.droneProfiles = new ArrayList<>();
        for (int i = 0; i < form.droneCount; i++)
            form.droneProfiles.add(typeOf(i));
        form.flightTimeS = searchTimeS;
        requestedDrone1 = typeOf(0);
        Calendar now = Calendar.getInstance();
        form.clockNowMin = now.get(Calendar.HOUR_OF_DAY) * 60 + now.get(Calendar.MINUTE);
    }

    private static double parse(EditText e, double def) {
        try {
            double v = Double.parseDouble(e.getText().toString().trim());
            return Double.isNaN(v) || v < 0 ? def : v;
        } catch (RuntimeException ex) {
            return def;
        }
    }

    // ---- planning -----------------------------------------------------------

    /** The connected drone's position (any thread), or null. */
    private LatLon dronePosition() {
        VehicleState st = s.link.getState();
        if (s.link.isVehicleConnected() && st.hasGlobalPosition)
            return LatLon.of(st.lat, st.lon);
        return null;
    }

    private void plan() {
        readFields();
        final List<double[]> area = searchAreaSamples();
        if (area.isEmpty() || s.dtedDownloader == null) {
            controller.plan(form); // the engine explains a missing search area
            return;
        }
        fetchingTerrain = true;
        statusText.setText("Fetching terrain for the search area…");
        terrainExec.execute(() -> {
            try {
                // best DTED the server has for every 1° cell under the area (the SERE prior reads
                // it); download progress goes to logcat, not the pane
                s.dtedDownloader.ensureCells(DtedDownloader.cellsFor(area),
                        msg -> android.util.Log.d(TAG, msg));
            } catch (InterruptedException e) {
                return;
            }
            android.util.Log.d(TAG, "Terrain under the search area: " + terrainSources(area));
            main.post(() -> {
                fetchingTerrain = false;
                controller.plan(form); // auto planner until the deadline
            });
        });
    }

    /** The terrain data the plan reads under the search area, e.g. "DTED0, DTED2" (for logcat). */
    private String terrainSources(List<double[]> area) {
        TreeSet<String> seen = new TreeSet<>();
        for (double[] p : area) {
            String src = s.elevation.sourceAt(p[0], p[1]);
            seen.add(src == null || src.isEmpty() ? "none" : src);
        }
        return android.text.TextUtils.join(", ", seen);
    }

    /** Points covering every shape titled with the search-area prefix ({lat, lon} each). */
    private List<double[]> searchAreaSamples() {
        List<double[]> out = new ArrayList<>();
        String prefix = cfg.searchAreaPrefix.toUpperCase(Locale.ROOT);
        for (TakMapAccess.Shape sh : map.shapes()) {
            boolean isArea = form.searchAreaUid != null ? form.searchAreaUid.equals(sh.uid)
                    : sh.title.trim().toUpperCase(Locale.ROOT).startsWith(prefix);
            if (!isArea)
                continue;
            double minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
            for (LatLon p : sh.points) {
                minLat = Math.min(minLat, p.lat);
                maxLat = Math.max(maxLat, p.lat);
                minLon = Math.min(minLon, p.lon);
                maxLon = Math.max(maxLon, p.lon);
            }
            for (double lat = minLat; ; lat = Math.min(maxLat, lat + AREA_SAMPLE_DEG)) {
                for (double lon = minLon; ; lon = Math.min(maxLon, lon + AREA_SAMPLE_DEG)) {
                    out.add(new double[] {lat, lon});
                    if (lon >= maxLon)
                        break;
                }
                if (lat >= maxLat)
                    break;
            }
        }
        return out;
    }

    private void showPlan(SearchPlan plan, boolean preview) {
        fullPlan = preview ? null : plan; // a preview replaces the plan on the map: plan again to fly
        if (!preview)
            planDrone1 = requestedDrone1;
        if (!preview && awaitingNextFlight) {
            awaitingNextFlight = false;
            nextFlightPlan = plan;
        }
        String kind = preview ? "Preview (press Plan search to refine)"
                : plan == nextFlightPlan ? "Flight " + (flightNumber + 1) + " plan"
                : plan.startTimeS > 0 ? "Re-plan" : "Plan";
        // a large area is planned on coarser cells (the probability map is that coarse)
        String cells = plan.grid.cellSizeM > cfg.cellSizeM + 0.5
                ? String.format(Locale.US, " · %.0f m cells (large area)", plan.grid.cellSizeM) : "";
        statusText.setText(String.format(Locale.US,
                "%s: chance of finding %.0f%% by the end · %s · %.1f s%s", kind,
                100 * plan.fleetCpdAtEnd, plan.planner, plan.runtimeMs / 1000.0, cells));

        StringBuilder sb = new StringBuilder();
        for (DroneRoute r : plan.routes) {
            if (sb.length() > 0)
                sb.append('\n');
            boolean linked = DRONE_ID.equals(r.droneId);
            sb.append(String.format(Locale.US, "%s  %3d wp  %4.1f min  finds %2.0f%%%s",
                    r.droneId, r.waypoints.size(), r.durationS / 60.0, 100 * r.cpd,
                    !linked ? "  (no link)" : canUpload() ? "" : "  (plan only)"));
        }
        routesText.setText(sb);

        // only a route that breaks its flight time (or a drone with no route) is worth the
        // operator's attention; the model's notes go to logcat
        List<String> warnings = PlanWarnings.shown(plan.warnings);
        for (String w : plan.warnings)
            if (!warnings.contains(w))
                android.util.Log.d(TAG, "plan note: " + w);
        StringBuilder warn = new StringBuilder();
        for (String w : warnings) {
            if (warn.length() > 0)
                warn.append('\n');
            warn.append("⚠ ").append(w);
        }
        warnText.setVisibility(warn.length() > 0 ? View.VISIBLE : View.GONE);
        warnText.setText(warn);
    }

    // ---- upload / fly -------------------------------------------------------

    private static DroneRoute route(SearchPlan p, String id) {
        for (DroneRoute r : p.routes)
            if (id.equals(r.droneId))
                return r;
        return null;
    }

    /** Whether the shown plan's drone-1 is a type the plugin flies over MAVLink (ArduCopter). */
    private boolean canUpload() {
        return planDrone1 != null && planDrone1.mavlinkUpload;
    }

    private boolean uploaded() {
        SearchPlan p = fullPlan;
        return p != null && uploadedPlan == p && s.missionOnVehicle == p;
    }

    private void upload() {
        final SearchPlan p = fullPlan;
        if (p == null || !s.link.isVehicleConnected() || !canUpload())
            return;
        DroneRoute r = route(p, DRONE_ID);
        if (r == null) {
            log("The plan has no route for " + DRONE_ID);
            return;
        }
        for (DroneRoute other : p.routes)
            if (other != r)
                log(other.droneId + ": no drone link for it, route not uploaded");
        final List<Waypoint> wps = SearchMissionBuilder.flatten(r.segments.isEmpty()
                ? Collections.singletonList(r.waypoints) : r.segments);
        if (s.link.getState().armed) {
            double over = replanOverrunS(p, wps.get(0));
            if (over > 0) {
                // what the operator reviewed would no longer fit the flight time: never fly it
                s.confirm("Re-plan again?", String.format(Locale.US,
                        "This re-plan no longer fits the flight time: the drone kept flying since "
                                + "Re-plan was pressed, and the route would overrun by about %.0f s."
                                + "\n\nRe-plan from where the drone is now, then upload right away.",
                        over), () -> controller.replan(System.currentTimeMillis()));
                return;
            }
            s.confirm("Upload in flight?",
                    "The drone will stop and hold position (GUIDED) while the new route uploads. "
                            + "Then press \"Fly new route\".",
                    () -> doUpload(p, wps, true));
        } else {
            doUpload(p, wps, false);
        }
    }

    /**
     * Seconds a re-planned route would overrun its flight time if uploaded now (0 or less: fits;
     * NEGATIVE_INFINITY for a first plan, which starts on the ground).
     */
    private double replanOverrunS(SearchPlan p, Waypoint routeStart) {
        SearchSession session = controller.session();
        if (p.startTimeS <= 0 || session == null)
            return Double.NEGATIVE_INFINITY;
        double since = (System.currentTimeMillis() - session.epochMs()) / 1000.0 - p.startTimeS;
        VehicleState st = s.link.getState();
        double back = st.hasGlobalPosition
                ? RoutePlanner.distance(st.lat, st.lon, routeStart.lat, routeStart.lon) : 0;
        return ArduCopterTiming.replanOverrunS(since, back, cfg.replanStartAllowanceS);
    }

    private void doUpload(final SearchPlan p, final List<Waypoint> wps, final boolean airborne) {
        s.runner.start("Search upload", this::log, ctx -> {
            if (airborne) {
                ctx.log("GUIDED → holding position");
                s.commands.setModeBlocking(CopterMode.GUIDED);
            }
            VehicleState st = s.link.getState();
            Waypoint first = wps.get(0);
            double startLat, startLon, homeAmsl;
            if (st.hasGlobalPosition) {
                startLat = st.lat;
                startLon = st.lon;
                homeAmsl = st.altAmsl - st.relAlt; // ArduPilot home altitude
                double off = RoutePlanner.distance(st.lat, st.lon, first.lat, first.lon);
                if (!airborne && off > LAUNCH_WARN_M)
                    ctx.log(String.format(Locale.US, "⚠ Drone is %.0f m from the plan's launch "
                            + "point; plan again from here for accurate timing", off));
            } else if (airborne) {
                throw new Exception("no drone position: cannot build a route from here");
            } else {
                startLat = first.lat;
                startLon = first.lon;
                homeAmsl = s.elevation.getElevationMsl(first.lat, first.lon);
                if (Double.isNaN(homeAmsl))
                    throw new RoutePlanner.NoTerrainException(first.lat, first.lon);
            }

            // terrain for every 1° cell the route touches (altitudes are set from it)
            if (s.dtedDownloader != null) {
                List<double[]> pts = new ArrayList<>(wps.size() + 1);
                pts.add(new double[] {startLat, startLon});
                for (Waypoint w : wps)
                    pts.add(new double[] {w.lat, w.lon});
                s.dtedDownloader.ensureCells(DtedDownloader.cellsFor(pts), ctx::log);
            }

            int sys = s.link.getTargetSystem(), comp = s.link.getTargetComponent();
            SearchMissionBuilder.Result m = SearchMissionBuilder.build(sys, comp, startLat,
                    startLon, homeAmsl, airborne, wps, s.elevation);
            ctx.log(String.format(Locale.US, "Uploading %d waypoints, up to %.0f m above home%s",
                    m.waypointCount, m.maxAltAboveHomeM, m.maxTerrainLiftM > 1
                            ? String.format(Locale.US, " (up to +%.0f m for terrain on legs)",
                                    m.maxTerrainLiftM)
                            : ""));
            new MissionUploader(s.link).upload(m.items, null);
            if (airborne) {
                // AUTO must resume at item 1, the climb in place that makes the first leg safe
                ctx.log("Setting the new route's first item");
                s.commands.setMissionCurrentBlocking(1);
            }
            uploadedPlan = p;
            s.missionOnVehicle = p;
        }, null);
    }

    private void confirmStart() {
        final SearchPlan p = fullPlan;
        if (!uploaded() || p == null)
            return;
        if (s.link.getState().armed) {
            s.confirm("Fly new route?", "The drone will switch to AUTO and fly the uploaded route, "
                    + "then land at the recovery point.", this::startFlight);
            return;
        }
        DroneRoute r = route(p, DRONE_ID);
        s.confirm("Start search?", String.format(Locale.US,
                "The drone will arm, take off, fly the %.0f-minute search route and land at the "
                        + "recovery point.\n\nIs the area clear?",
                r != null ? r.durationS / 60.0 : 0), this::startFlight);
    }

    private void startFlight() {
        long now = System.currentTimeMillis();
        s.flight.started(now);
        if (!missionRunning) {
            controller.startMission(now); // in-flight belief, time 0 at launch
            missionRunning = true;
            lastReportMs = 0;
            flightNumber = 1;
            airborneS = 0;
            lastAirborneTickMs = 0;
            updateDroneTypes();
        } else if (nextFlightPlan != null && fullPlan == nextFlightPlan) {
            controller.startNextFlight(now); // its route's time counts from this takeoff
            nextFlightPlan = null;
            flightNumber++;
        }
        s.runner.start("Search flight", this::log, ctx -> {
            VehicleState st = s.link.getState();
            if (!st.armed) {
                ctx.log("GUIDED → arming");
                s.commands.setModeBlocking(CopterMode.GUIDED);
                s.commands.armBlocking(true);
            }
            ctx.log("AUTO → search route");
            s.commands.setModeBlocking(CopterMode.AUTO);
            s.commands.missionStartBlocking();
        }, null);
    }

    /** Search time not flown yet, s. */
    private double searchTimeLeftS() {
        return Math.max(0, searchTimeS - airborneS);
    }

    /**
     * Next flight: plans a fresh battery's route over what is left, for the search time not flown
     * yet (capped at each drone's time in the air).
     */
    private void nextFlight() {
        readFields();
        double left = searchTimeLeftS();
        if (left < MIN_NEXT_FLIGHT_S) {
            s.toast("The search time is used up: raise it to fly again, or End search");
            return;
        }
        form.flightTimeS = left;
        awaitingNextFlight = true;
        log(String.format(Locale.US, "Planning flight %d over the area not searched yet (%s of "
                + "search time left)", flightNumber + 1, hoursMinutes(left)));
        controller.planNextFlight(form, System.currentTimeMillis());
    }

    private void endMission() {
        missionRunning = false;
        updateDroneTypes();
        controller.endMission();
        beliefText.setText("");
        overlay.clearAll();
        SearchPlan p = fullPlan;
        if (p != null)
            overlay.drawPlan(p, false);
    }

    // ---- state → UI -----------------------------------------------------------

    /** Called by the UI ticker. */
    void refresh(boolean flightActive) {
        boolean connected = s.link.isVehicleConnected();
        boolean busy = s.runner.isBusy();
        boolean thinking = planning || fetchingTerrain;
        long now = System.currentTimeMillis();
        if (now - lastMapTextMs >= MAP_TEXT_REFRESH_MS)
            updateMapInputs(true); // the operator may move or delete map items in ATAK

        if (missionRunning)
            feedTelemetry(connected, now);
        boolean armed = connected && s.link.getState().armed;
        trackAirborne(armed, now);

        boolean uploaded = uploaded();
        setEnabled(planBtn, !thinking && !busy);
        setEnabled(cancelBtn, planning);
        setEnabled(uploadBtn, fullPlan != null && canUpload() && connected && !thinking && !busy
                && !uploaded);
        setEnabled(startBtn, uploaded && connected && !busy);
        setEnabled(replanBtn, missionRunning && armed && !thinking && !busy);
        // after a landing: a fresh battery's route over what is not searched yet
        setEnabled(nextFlightBtn, missionRunning && !armed && !flightActive && !thinking && !busy);
        setEnabled(endBtn, missionRunning);
        // in flight the area and red zones are what re-plans read: End search first
        setEnabled(resetBtn, !missionRunning && !thinking);
        boolean groundStart = !armed && (!missionRunning || fullPlan == nextFlightPlan);
        startBtn.setText(groundStart ? R.string.search_start : R.string.search_fly_new);

        String hint;
        if (fullPlan == null)
            hint = "Draw or pick the search area, then Plan search";
        else if (!canUpload())
            hint = planDrone1 == null ? "Plan search again"
                    : "Plan only: fly the " + planDrone1.name + " route from its own ground "
                            + "station (the plugin uploads to ArduCopter only)";
        else if (!connected)
            hint = "Connect a drone first (⚙)";
        else if (busy)
            hint = "Working…";
        else if (!uploaded && armed && missionRunning && fullPlan.startTimeS > 0)
            hint = replanHint(fullPlan);
        else if (!uploaded)
            hint = "Uploads drone-1's route with terrain-safe altitudes";
        else if (flightActive)
            hint = "Use R2Home, Land or KILL above";
        else if (missionRunning)
            hint = searchTimeLeftS() < MIN_NEXT_FLIGHT_S
                    ? "Landed: the search time is used up. Raise it to fly again, or End search"
                    : "Landed: " + hoursMinutes(searchTimeLeftS()) + " of search time left. Swap "
                            + "the battery, then Next flight continues with the rest of the area";
        else
            hint = "Route uploaded ✓";
        hintText.setText(hint);
    }

    /** Adds the time the drone is in the air to the search time flown. */
    private void trackAirborne(boolean armed, long now) {
        if (missionRunning && armed) {
            if (lastAirborneTickMs > 0)
                airborneS += (now - lastAirborneTickMs) / 1000.0;
            lastAirborneTickMs = now;
        } else {
            lastAirborneTickMs = 0;
        }
        int min = missionRunning ? (int) (airborneS / 60) : -1;
        if (min != shownAirborneMin) {
            shownAirborneMin = min;
            updateSearchTimeLabel();
        }
    }

    /** How long the operator has to upload a re-planned route before it stops fitting. */
    private String replanHint(SearchPlan p) {
        DroneRoute r = route(p, DRONE_ID);
        if (r == null || r.waypoints.isEmpty())
            return "";
        double over = replanOverrunS(p, r.waypoints.get(0));
        if (over > 0)
            return "This re-plan no longer fits the flight time: Re-plan again";
        return String.format(Locale.US, "Upload within %.0f s, or the route will not fit the "
                + "flight time", -over);
    }

    /** Drone-1's position into the belief once a second; a timed redraw while it is silent. */
    private void feedTelemetry(boolean connected, final long now) {
        VehicleState st = s.link.getState();
        if (connected && st.hasGlobalPosition) {
            if (now - lastReportMs < REPORT_MS || !telemetryBusy.compareAndSet(false, true))
                return;
            lastReportMs = now;
            final double lat = st.lat, lon = st.lon, amsl = st.altAmsl;
            final boolean sensorOn = st.armed; // camera assumed on whenever the drone flies
            runTelemetry(() -> {
                double ground = s.elevation.getElevationMsl(lat, lon);
                double agl = Double.isNaN(ground) ? Double.NaN : amsl - ground;
                controller.onDronePosition(DRONE_ID, lat, lon, agl, now, sensorOn);
            });
        } else if (now - lastTimerRefreshMs >= cfg.beliefRedrawMs
                && telemetryBusy.compareAndSet(false, true)) {
            lastTimerRefreshMs = now; // the person keeps moving while the drone is silent
            runTelemetry(() -> controller.refreshBelief(now));
        }
    }

    /** Runs one telemetry update; the caller has set {@link #telemetryBusy}. */
    private void runTelemetry(Runnable update) {
        try {
            telemetryExec.execute(() -> {
                try {
                    update.run();
                } finally {
                    telemetryBusy.set(false);
                }
            });
        } catch (RuntimeException e) { // executor shut down
            telemetryBusy.set(false);
        }
    }

    private static void setEnabled(View v, boolean enabled) {
        v.setEnabled(enabled);
        v.setAlpha(enabled ? 1f : 0.45f);
    }

    void dispose() {
        tools.dispose();
        missionRunning = false;
        terrainExec.shutdownNow();
        telemetryExec.shutdownNow();
        controller.close();
        overlay.clearAll();
    }
}
