package com.atakmap.android.plugintemplate.plugin.ui;

import android.app.AlertDialog;
import android.content.Context;
import android.graphics.PointF;
import android.text.InputType;
import android.util.Log;
import android.util.TypedValue;
import android.view.WindowManager;
import android.view.inputmethod.EditorInfo;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;

import com.atakmap.android.drawing.mapItems.DrawingCircle;
import com.atakmap.android.drawing.mapItems.DrawingShape;
import com.atakmap.android.maps.MapEvent;
import com.atakmap.android.maps.MapEventDispatcher;
import com.atakmap.android.maps.MapGroup;
import com.atakmap.android.maps.MapItem;
import com.atakmap.android.maps.MapView;
import com.atakmap.android.maps.Polyline;
import com.atakmap.android.user.PlacePointTool;
import com.atakmap.android.util.ATAKUtilities;
import com.atakmap.coremap.conversions.CoordinateFormat;
import com.atakmap.coremap.conversions.CoordinateFormatUtilities;
import com.atakmap.coremap.maps.coords.Ellipsoid;
import com.atakmap.coremap.maps.coords.GeoPoint;
import com.atakmap.coremap.maps.coords.GeoPointMetaData;
import com.atakmap.coremap.maps.coords.MGRSPoint;
import com.mindfulpigeon.atak.bridge.PluginConfig;
import com.mindfulpigeon.atak.bridge.TakMapAccess;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.UUID;

/**
 * The Search tab's map tools. The operator can draw the search area or a red zone (each tap adds a
 * corner and the shape fills in from the third), pick an existing shape as the search area, and
 * place the last known position (LKP) or a suspected location with a tap or by MGRS grid. A red
 * zone can also be a circle around an MGRS grid. What the tools make are ordinary TAK items (shapes
 * in Drawing Objects, spot markers) titled by the plugin's naming conventions, so they are shared,
 * edited and deleted like any other.
 *
 * <p>ATAK's Shape and Marker are written out in full: the TakMapAccess interface's nested Shape and
 * Marker types share their names.
 */
final class SearchMapTools {

    /** What a map tap does. */
    enum Mode {
        NONE, PICK_AREA, DRAW_AREA, DRAW_RED_ZONE, PLACE_LKP, PLACE_SUSPECT, PLACE_LAUNCH;

        boolean drawing() {
            return this == DRAW_AREA || this == DRAW_RED_ZONE;
        }
    }

    interface Listener {
        /** The search area is now this shape (picked or drawn). */
        void searchArea(String uid, String title);

        /** This marker is now a suspected location (placed or picked). */
        void suspect(String uid, String title);

        /** The LKP marker, a red zone or the drawing changed: refresh what the tab shows. */
        void changed();
    }

    private static final String TAG = "KoalifiedSearchTools";
    /** The plugin's draft group: the shape being drawn, never read as a TAK shape. */
    static final String GROUP_DRAFT = "P.U.L.S.E. Search draft";
    private static final String DRAWING_GROUP = "Drawing Objects";
    private static final String SPOT_TYPE = "b-m-p-s-m";
    private static final String SUSPECT_PREFIX = "Spot";

    private static final int AREA_STROKE = 0xFFFFEB3B;
    private static final int AREA_FILL = 0x33FFEB3B;
    private static final int RED_ZONE_STROKE = 0xFFE53935;
    private static final int RED_ZONE_FILL = 0x44E53935;
    private static final int LKP_COLOR = 0xFFFF9800;
    private static final int SUSPECT_COLOR = 0xFFE040FB;
    private static final int LAUNCH_COLOR = 0xFF00C853;
    /** ATAK's preferences file holding the UIDs of what this tab placed (for Reset). */
    private static final String PREFS = "koalified_search_tools";
    private static final String KEY_PLACED = "placed";
    /** Half the side of the view an MGRS entry zooms to, metres. */
    private static final double FOCUS_HALF_M = 300;
    private static final double DEFAULT_RED_ZONE_RADIUS_M = 200;

    private final Services s;
    private final MapView view;
    private final PluginConfig cfg;
    private final TakMapAccess map;
    private final Listener listener;

    private Mode mode = Mode.NONE;
    private final ShapeDraft draft = new ShapeDraft();
    private final List<MapItem> draftItems = new ArrayList<>();
    private MapGroup draftGroup;
    /** UIDs of the shapes and markers this tab made, oldest first. */
    private final java.util.LinkedHashSet<String> placed = new java.util.LinkedHashSet<>();

    SearchMapTools(Services s, PluginConfig cfg, TakMapAccess map, Listener listener) {
        this.s = s;
        this.view = s.mapView;
        this.cfg = cfg;
        this.map = map;
        this.listener = listener;
        loadPlaced();
    }

    Mode mode() {
        return mode;
    }

    /** Corners drawn so far (0 when not drawing). */
    int corners() {
        return mode.drawing() ? draft.size() : 0;
    }

    boolean canFinish() {
        return mode.drawing() && draft.canFinish();
    }

    // ---- modes -----------------------------------------------------------------

    /** Starts a tool; starting the one that is on turns it off. */
    void toggle(Mode m) {
        if (mode == m) {
            stop();
            return;
        }
        stop();
        if (view == null || m == Mode.NONE)
            return;
        MapEventDispatcher d = view.getMapEventDispatcher();
        d.pushListeners();
        d.clearListeners(MapEvent.MAP_CLICK);
        d.clearListeners(MapEvent.ITEM_CLICK);
        MapEventDispatcher.MapEventDispatchListener l = this::onTap;
        d.addMapEventListener(MapEvent.MAP_CLICK, l);
        d.addMapEventListener(MapEvent.ITEM_CLICK, l);
        mode = m;
        switch (m) {
            case PICK_AREA:
                s.toast("Tap the shape to search");
                break;
            case DRAW_AREA:
                s.toast("Tap the map at each corner of the search area, then Done");
                break;
            case DRAW_RED_ZONE:
                s.toast("Tap the map at each corner of the red zone, then Done");
                break;
            case PLACE_LKP:
                s.toast("Tap the last known position");
                break;
            case PLACE_SUSPECT:
                s.toast("Tap where the person may be, or a marker already there");
                break;
            case PLACE_LAUNCH:
                s.toast("Tap where the drone takes off");
                break;
            default:
                break;
        }
        listener.changed();
    }

    /** Turns the tool off; a shape being drawn is discarded. */
    void stop() {
        if (mode == Mode.NONE)
            return;
        mode = Mode.NONE;
        if (view != null)
            view.getMapEventDispatcher().popListeners();
        while (draft.undo()) {
            // discard the corners
        }
        redrawDraft();
        listener.changed();
    }

    void undoCorner() {
        if (mode.drawing() && draft.undo()) {
            redrawDraft();
            listener.changed();
        }
    }

    /** Makes the drawn shape a TAK shape and turns the tool off. */
    void finishShape() {
        if (!canFinish()) {
            s.toast("Tap at least " + ShapeDraft.MIN_CORNERS + " corners first");
            return;
        }
        Mode was = mode;
        List<double[]> corners = draft.corners();
        stop();
        GeoPoint[] pts = new GeoPoint[corners.size()];
        for (int i = 0; i < pts.length; i++)
            pts[i] = new GeoPoint(corners.get(i)[0], corners.get(i)[1]);
        if (was == Mode.DRAW_AREA) {
            // titled like the search-area convention, so it is still found after a restart
            String title = MapNames.next(cfg.searchAreaPrefix, shapeTitles());
            DrawingShape shape = addShape(pts, title, AREA_STROKE, AREA_FILL);
            listener.searchArea(shape.getUID(), title);
        } else {
            addShape(pts, MapNames.next(cfg.redZonePrefix, shapeTitles()), RED_ZONE_STROKE,
                    RED_ZONE_FILL);
        }
        listener.changed();
    }

    void dispose() {
        stop();
        clearDraft();
    }

    // ---- taps ----------------------------------------------------------------------

    private void onTap(MapEvent event) {
        MapItem item = MapEvent.ITEM_CLICK.equals(event.getType()) ? event.getItem() : null;
        switch (mode) {
            case PICK_AREA:
                stop();
                pickArea(item);
                break;
            case DRAW_AREA:
            case DRAW_RED_ZONE: {
                GeoPoint p = pointOf(event);
                if (p != null && draft.add(p.getLatitude(), p.getLongitude())) {
                    redrawDraft();
                    listener.changed();
                }
                break;
            }
            case PLACE_LKP: {
                GeoPoint p = pointOf(event);
                stop();
                if (p != null)
                    placeLkp(p, false);
                break;
            }
            case PLACE_LAUNCH: {
                GeoPoint p = pointOf(event);
                stop();
                if (p != null)
                    placeLaunch(p, false);
                break;
            }
            case PLACE_SUSPECT: {
                if (item instanceof com.atakmap.android.maps.Marker) {
                    stop(); // a marker already there: it becomes the suspected location
                    listener.suspect(item.getUID(), title(item));
                    break;
                }
                GeoPoint p = pointOf(event);
                stop();
                if (p != null)
                    placeSuspect(p, false);
                break;
            }
            default:
                break;
        }
    }

    private GeoPoint pointOf(MapEvent event) {
        PointF pt = event.getPointF();
        GeoPointMetaData gp = pt != null ? view.inverseWithElevation(pt.x, pt.y) : null;
        if (gp == null || gp.get() == null || !gp.get().isValid()) {
            s.toast("Could not read that map location");
            return null;
        }
        return gp.get();
    }

    private void pickArea(MapItem item) {
        com.atakmap.android.maps.Shape shape = item != null ? shapeOf(item) : null;
        if (shape == null) {
            s.toast("No shape there: tap the outline of a polygon, rectangle or circle");
            return;
        }
        GeoPoint[] pts = shape.getPoints();
        if (pts == null || pts.length < 3) {
            s.toast("'" + title(shape) + "' is not a closed shape");
            return;
        }
        listener.searchArea(shape.getUID(), title(shape));
    }

    /** The shape a tapped item stands for: the shape itself, or the shape its centre marker belongs to. */
    private com.atakmap.android.maps.Shape shapeOf(MapItem item) {
        if (item instanceof com.atakmap.android.maps.Shape)
            return (com.atakmap.android.maps.Shape) item;
        String uid = item.getMetaString("shapeUID", null);
        if (uid != null) {
            MapItem parent = view.getRootGroup().deepFindUID(uid);
            if (parent instanceof com.atakmap.android.maps.Shape)
                return (com.atakmap.android.maps.Shape) parent;
        }
        return null;
    }

    /** The plugin's naming convention: a title starting with the prefix, case ignored. */
    static boolean titled(String title, String prefix) {
        return title != null && title.trim().toUpperCase(Locale.ROOT)
                .startsWith(prefix.toUpperCase(Locale.ROOT));
    }

    static String title(MapItem item) {
        String t = item.getTitle();
        return t == null || t.trim().isEmpty() ? "Untitled" : t.trim();
    }

    // ---- making TAK items ----------------------------------------------------------

    private List<String> shapeTitles() {
        List<String> out = new ArrayList<>();
        for (TakMapAccess.Shape sh : map.shapes())
            out.add(sh.title);
        return out;
    }

    private List<String> markerTitles() {
        List<String> out = new ArrayList<>();
        for (TakMapAccess.Marker m : map.markers())
            out.add(m.title);
        return out;
    }

    private MapGroup drawingGroup() {
        MapGroup root = view.getRootGroup();
        MapGroup g = root.findMapGroup(DRAWING_GROUP);
        return g != null ? g : root.addGroup(DRAWING_GROUP);
    }

    private DrawingShape addShape(GeoPoint[] pts, String title, int stroke, int fill) {
        DrawingShape shape = new DrawingShape(view, UUID.randomUUID().toString());
        shape.setPoints(pts);
        shape.setClosed(true); // before the fill: DrawingShape fills closed shapes only
        shape.setTitle(title);
        shape.setStrokeColor(stroke);
        shape.setFillColor(fill);
        shape.setStrokeWeight(3d);
        drawingGroup().addItem(shape);
        shape.persist(view.getMapEventDispatcher(), null, getClass());
        remember(shape);
        return shape;
    }

    private void addRedZoneCircle(GeoPoint centre, double radiusM) {
        String title = MapNames.next(cfg.redZonePrefix, shapeTitles());
        DrawingCircle c = new DrawingCircle(view, UUID.randomUUID().toString());
        c.setCenterPoint(GeoPointMetaData.wrap(centre));
        c.setRadius(radiusM);
        c.setTitle(title);
        c.setStrokeColor(RED_ZONE_STROKE);
        c.setFillColor(RED_ZONE_FILL);
        c.setStrokeWeight(3d);
        drawingGroup().addItem(c);
        c.persist(view.getMapEventDispatcher(), null, getClass());
        remember(c);
        focus(centre, Math.max(FOCUS_HALF_M, 1.5 * radiusM));
        s.toast(title + " added");
        listener.changed();
    }

    /** Puts the LKP here (see {@link #placeOne}). */
    private void placeLkp(GeoPoint p, boolean focus) {
        placeOne(cfg.lkpPrefix, cfg.lkpPrefix, LKP_COLOR, p, focus);
    }

    /** Puts drone-1's launch point here (see {@link #placeOne}). */
    private void placeLaunch(GeoPoint p, boolean focus) {
        placeOne(cfg.launchPrefix, cfg.launchPrefix + " 1", LAUNCH_COLOR, p, focus);
    }

    /**
     * Moves the marker the plan reads for a one-of-a-kind point (the first one titled with the
     * prefix), or places a new one titled newTitle when the map has none.
     */
    private void placeOne(String prefix, String newTitle, int color, GeoPoint p, boolean focus) {
        com.atakmap.android.maps.Marker existing = null;
        for (TakMapAccess.Marker m : map.markers()) {
            if (titled(m.title, prefix)) {
                MapItem item = view.getRootGroup().deepFindUID(m.uid);
                if (item instanceof com.atakmap.android.maps.Marker) {
                    existing = (com.atakmap.android.maps.Marker) item;
                    break;
                }
            }
        }
        if (existing != null) {
            existing.setPoint(p);
            existing.persist(view.getMapEventDispatcher(), null, getClass());
        } else {
            spot(p, newTitle, color);
        }
        if (focus)
            focus(p, FOCUS_HALF_M);
        listener.changed();
    }

    /** The device's own position (ATAK's self marker: GPS, or a position set by hand), or null. */
    private GeoPoint myLocation() {
        com.atakmap.android.maps.Marker self = view != null ? view.getSelfMarker() : null;
        GeoPoint p = self != null ? self.getPoint() : null;
        if (p == null || !p.isValid() || (p.getLatitude() == 0 && p.getLongitude() == 0))
            return null;
        return p;
    }

    void launchAtMyLocation() {
        stop();
        GeoPoint p = myLocation();
        if (p == null) {
            s.toast("No current location yet: ATAK has no GPS fix or self position");
            return;
        }
        placeLaunch(p, true);
        s.toast("Launch point set to your current location");
    }

    // ---- what this tab placed, for Reset -------------------------------------------

    /** Remembers an item this tab made, across restarts (ATAK's preferences: the plugin has none). */
    private void remember(MapItem item) {
        if (item == null)
            return;
        placed.add(item.getUID());
        savePlaced();
    }

    private void savePlaced() {
        try {
            s.hostContext().getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit()
                    .putString(KEY_PLACED, android.text.TextUtils.join(",", placed)).apply();
        } catch (RuntimeException e) {
            Log.w(TAG, "could not save the placed items", e);
        }
    }

    private void loadPlaced() {
        try {
            String v = s.hostContext().getSharedPreferences(PREFS, Context.MODE_PRIVATE)
                    .getString(KEY_PLACED, "");
            for (String uid : v.split(","))
                if (!uid.trim().isEmpty())
                    placed.add(uid.trim());
        } catch (RuntimeException e) {
            Log.w(TAG, "could not read the placed items", e);
        }
    }

    /** The items this tab placed that are still on the map. */
    List<MapItem> placedItems() {
        List<MapItem> out = new ArrayList<>();
        if (view == null)
            return out;
        for (String uid : placed) {
            MapItem item = view.getRootGroup().deepFindUID(uid);
            if (item != null)
                out.add(item);
        }
        return out;
    }

    /** Removes from the map everything this tab placed. @return how many items were removed */
    int removePlaced() {
        stop();
        int n = 0;
        for (MapItem item : placedItems())
            if (item.removeFromGroup())
                n++;
        placed.clear();
        savePlaced();
        listener.changed();
        return n;
    }

    private void placeSuspect(GeoPoint p, boolean focus) {
        String title = MapNames.next(SUSPECT_PREFIX, markerTitles());
        com.atakmap.android.maps.Marker m = spot(p, title, SUSPECT_COLOR);
        if (focus)
            focus(p, FOCUS_HALF_M);
        if (m != null)
            listener.suspect(m.getUID(), title);
    }

    private com.atakmap.android.maps.Marker spot(GeoPoint p, String title, int color) {
        com.atakmap.android.maps.Marker m = new PlacePointTool.MarkerCreator(p)
                .setUid(UUID.randomUUID().toString())
                .setType(SPOT_TYPE)
                .setCallsign(title)
                .setColor(color)
                .showCotDetails(false)
                .placePoint();
        remember(m);
        return m;
    }

    /** Zooms the map to a square of the given half side around a point. */
    private void focus(GeoPoint p, double halfM) {
        double dLat = halfM / 111_320.0;
        double dLon = dLat / Math.max(0.01, Math.cos(Math.toRadians(p.getLatitude())));
        GeoPoint[] box = {
                new GeoPoint(p.getLatitude() - dLat, p.getLongitude() - dLon),
                new GeoPoint(p.getLatitude() + dLat, p.getLongitude() + dLon)};
        ATAKUtilities.scaleToFit(view, box, view.getWidth(), view.getHeight());
    }

    // ---- the shape being drawn -----------------------------------------------------

    private void redrawDraft() {
        clearDraft();
        List<double[]> corners = draft.corners();
        if (corners.isEmpty() || view == null)
            return;
        MapGroup g = draftGroup();
        boolean redZone = mode == Mode.DRAW_RED_ZONE;
        GeoPoint[] pts = new GeoPoint[corners.size()];
        for (int i = 0; i < pts.length; i++)
            pts[i] = new GeoPoint(corners.get(i)[0], corners.get(i)[1]);
        if (pts.length >= 2) {
            Polyline line = new Polyline(UUID.randomUUID().toString());
            line.setPoints(GeoPointMetaData.wrap(pts));
            line.setStrokeColor(redZone ? RED_ZONE_STROKE : AREA_STROKE);
            line.setStrokeWeight(3d);
            if (pts.length >= ShapeDraft.MIN_CORNERS) {
                line.setStyle(line.getStyle() | Polyline.STYLE_CLOSED_MASK
                        | com.atakmap.android.maps.Shape.STYLE_FILLED_MASK);
                line.setFillColor(redZone ? RED_ZONE_FILL : AREA_FILL);
            }
            local(line);
            g.addItem(line);
            draftItems.add(line);
        }
        for (int i = 0; i < pts.length; i++) {
            com.atakmap.android.maps.Marker m = new com.atakmap.android.maps.Marker(
                    UUID.randomUUID().toString());
            m.setPoint(pts[i]);
            m.setType("b-m-p-w"); // a dot per corner, unlabelled: outline order is not tap order
            m.setMovable(false);
            local(m);
            g.addItem(m);
            draftItems.add(m);
        }
    }

    /** Draft items: not sent as CoT, not listed, not editable, and taps go through to the map. */
    private static void local(MapItem item) {
        item.setClickable(false);
        item.setMetaBoolean("nevercot", true);
        item.setMetaBoolean("addToObjList", false);
        item.setMetaBoolean("editable", false);
    }

    private void clearDraft() {
        if (draftGroup != null)
            for (MapItem i : draftItems)
                draftGroup.removeItem(i);
        draftItems.clear();
    }

    private MapGroup draftGroup() {
        if (draftGroup == null) {
            MapGroup root = view.getRootGroup();
            draftGroup = root.findMapGroup(GROUP_DRAFT);
            if (draftGroup == null)
                draftGroup = root.addGroup(GROUP_DRAFT);
        }
        return draftGroup;
    }

    // ---- MGRS entry ------------------------------------------------------------------

    void enterLkpMgrs() {
        stop();
        askMgrs("Last known position (MGRS)", false, (p, r) -> placeLkp(p, true));
    }

    void enterSuspectMgrs() {
        stop();
        askMgrs("Suspected location (MGRS)", false, (p, r) -> placeSuspect(p, true));
    }

    void enterRedZoneMgrs() {
        stop();
        askMgrs("Red zone centre (MGRS)", true, this::addRedZoneCircle);
    }

    void enterLaunchMgrs() {
        stop();
        askMgrs("Launch point (MGRS)", false, (p, r) -> placeLaunch(p, true));
    }

    private interface GridEntered {
        void at(GeoPoint p, double radiusM);
    }

    /**
     * Asks for an MGRS grid, starting from the zone and 100 km square of the map centre so most
     * entries need only the digits. Stays open on a grid it cannot read.
     */
    private void askMgrs(String title, boolean withRadius, GridEntered onOk) {
        if (view == null)
            return;
        Context ctx = s.hostContext();
        int pad = dp(ctx, 16);
        LinearLayout box = new LinearLayout(ctx);
        box.setOrientation(LinearLayout.VERTICAL);
        box.setPadding(pad, dp(ctx, 8), pad, 0);

        String centre = "";
        GeoPointMetaData c = view.getCenterPoint();
        if (c != null && c.get() != null && c.get().isValid())
            centre = CoordinateFormatUtilities.formatToString(c.get(), CoordinateFormat.MGRS);
        // without ATAK's invisible left-to-right marks, which would end up in the typed grid
        centre = centre.replaceAll("\\p{Cf}", "").trim();
        String[] parts = centre.split(MgrsText.SPACES);
        String prefill = parts.length >= 2 ? parts[0] + " " + parts[1] + " " : "";

        TextView hint = new TextView(ctx);
        hint.setText("Grid with 6, 8 or 10 digits: 123 456 (100 m), 1234 5678 (10 m), "
                + "12345 67890 (1 m)." + (centre.isEmpty() ? ""
                        : "\nThe map centre is " + centre + "."));
        box.addView(hint);
        final EditText grid = new EditText(ctx);
        grid.setSingleLine(true);
        grid.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_FLAG_CAP_CHARACTERS
                | InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
        grid.setText(prefill);
        grid.setSelection(grid.getText().length());
        box.addView(grid);

        final EditText radius = new EditText(ctx);
        if (withRadius) {
            TextView rl = new TextView(ctx);
            rl.setText("Radius (m)");
            rl.setPadding(0, dp(ctx, 8), 0, 0);
            box.addView(rl);
            radius.setSingleLine(true);
            radius.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL);
            radius.setText(String.valueOf((int) DEFAULT_RED_ZONE_RADIUS_M));
            box.addView(radius);
        }

        final AlertDialog dialog = new AlertDialog.Builder(ctx)
                .setTitle(title)
                .setView(box)
                .setPositiveButton("Place", null)
                .setNegativeButton("Cancel", null)
                .create();
        // the Enter key can report its action twice: place once
        final boolean[] placed = {false};
        final Runnable submit = () -> {
            if (placed[0])
                return;
            GeoPoint p = parseMgrs(grid.getText().toString());
            if (p == null) {
                grid.setError("Not a grid: zone, band and 100 km square, then 6, 8 or 10 "
                        + "digits split evenly, e.g. 11S PU 362 447");
                grid.requestFocus();
                return;
            }
            double r = DEFAULT_RED_ZONE_RADIUS_M;
            if (withRadius) {
                try {
                    r = Double.parseDouble(radius.getText().toString().trim());
                } catch (NumberFormatException e) {
                    r = -1;
                }
                if (!(r > 0 && r <= 50_000)) {
                    radius.setError("Radius in metres, up to 50 000");
                    radius.requestFocus();
                    return;
                }
            }
            placed[0] = true;
            dialog.dismiss();
            onOk.at(p, r);
        };
        dialog.setOnShowListener(d -> dialog.getButton(AlertDialog.BUTTON_POSITIVE)
                .setOnClickListener(v -> submit.run()));
        // the keyboard covers the dialog's buttons in landscape: its ✓ key places too
        int keep = EditorInfo.IME_FLAG_NO_FULLSCREEN | EditorInfo.IME_FLAG_NO_EXTRACT_UI;
        grid.setImeOptions(keep | (withRadius ? EditorInfo.IME_ACTION_NEXT
                : EditorInfo.IME_ACTION_DONE));
        radius.setImeOptions(keep | EditorInfo.IME_ACTION_DONE);
        android.widget.TextView.OnEditorActionListener done = (v, action, ev) -> {
            if (action != EditorInfo.IME_ACTION_DONE)
                return false;
            submit.run();
            return true;
        };
        grid.setOnEditorActionListener(done);
        radius.setOnEditorActionListener(done);
        if (dialog.getWindow() != null)
            dialog.getWindow().setSoftInputMode(
                    WindowManager.LayoutParams.SOFT_INPUT_STATE_VISIBLE);
        dialog.show();
    }

    /** The point of an MGRS grid, or null when it is not one. */
    private static GeoPoint parseMgrs(String text) {
        String grid = MgrsText.canonical(text);
        if (grid == null) {
            Log.w(TAG, "not an MGRS grid: '" + text + "' " + codes(text));
            return null;
        }
        String[] parts = grid.split(" ");
        String[] tries = {grid, grid.replace(" ", ""),
                parts[0] + parts[1] + " " + parts[2] + " " + parts[3]};
        for (String t : tries) {
            try {
                GeoPoint p = CoordinateFormatUtilities.convert(t, CoordinateFormat.MGRS);
                if (p != null && p.isValid())
                    return p;
                Log.w(TAG, "MGRS '" + t + "' converted to " + p);
            } catch (RuntimeException e) {
                Log.w(TAG, "MGRS '" + t + "' failed", e);
            }
        }
        try {
            double[] ll = MGRSPoint.decode(parts[0], parts[1], parts[2], parts[3],
                    Ellipsoid.WGS_84, null).toLatLng(null);
            GeoPoint p = new GeoPoint(ll[0], ll[1]);
            if (p.isValid())
                return p;
            Log.w(TAG, "MGRS decode of '" + grid + "' gave " + ll[0] + ", " + ll[1]);
        } catch (RuntimeException e) {
            Log.w(TAG, "MGRS decode of '" + grid + "' failed", e);
        }
        return null;
    }

    /** The characters outside printable ASCII, as code points (hidden characters in typed text). */
    private static String codes(String text) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; text != null && i < text.length(); i++) {
            char c = text.charAt(i);
            if (c < 0x20 || c > 0x7E)
                sb.append(String.format(Locale.US, "U+%04X@%d ", (int) c, i));
        }
        return sb.toString();
    }

    /** "10T EK 56282 73114" for a point, as ATAK shows it. */
    static String mgrs(GeoPoint p) {
        try {
            return CoordinateFormatUtilities.formatToString(p, CoordinateFormat.MGRS);
        } catch (RuntimeException e) {
            return String.format(Locale.US, "%.5f, %.5f", p.getLatitude(), p.getLongitude());
        }
    }

    private static int dp(Context ctx, int v) {
        return (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, v,
                ctx.getResources().getDisplayMetrics());
    }
}
