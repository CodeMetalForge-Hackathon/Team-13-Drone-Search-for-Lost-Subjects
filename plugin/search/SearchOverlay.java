package com.atakmap.android.plugintemplate.plugin.search;

import android.graphics.Color;

import com.atakmap.android.maps.MapGroup;
import com.atakmap.android.maps.MapView;
import com.atakmap.android.maps.Polyline;
import com.atakmap.android.maps.Shape;
import com.atakmap.coremap.maps.coords.GeoPoint;
import com.mindfulpigeon.atak.bridge.PluginPorts;
import com.mindfulpigeon.search.api.BeliefSummary;
import com.mindfulpigeon.search.api.DroneRoute;
import com.mindfulpigeon.search.api.SearchPlan;
import com.mindfulpigeon.search.api.Waypoint;
import com.mindfulpigeon.search.geo.LatLon;
import com.mindfulpigeon.search.geo.LocalPlane;
import com.mindfulpigeon.search.prior.ProbabilityGrid;

import java.util.Arrays;
import java.util.UUID;

/**
 * Draws search plans on the TAK map: one polyline per drone route (dashed while previewing) and
 * the prior heatmap as its highest cells drawn as filled squares. In flight, the belief (where the
 * person probably is now) and the searched cells (cumulative POD of at least 0.5) are redrawn in a
 * group of their own. Main thread only (the controller calls it through the UI executor).
 */
public final class SearchOverlay implements PluginPorts.Overlay {

    public static final String GROUP_PLAN = "P.U.L.S.E. Search";
    public static final String GROUP_BELIEF = "P.U.L.S.E. Search belief";

    private static final int[] ROUTE_COLORS = {Color.CYAN, Color.GREEN, Color.MAGENTA, Color.YELLOW,
            Color.WHITE};
    private static final int HEAT_RGB = Color.rgb(255, 80, 0);
    private static final int SEARCHED_ARGB = Color.argb(60, 120, 200, 255);
    /** Heatmap squares per drawing; every square is a map item, so keep this phone-sized. */
    private static final int HEATMAP_CELLS = 400;
    /** Searched-cell squares per belief redraw. */
    private static final int SEARCHED_CELLS = 1000;

    public interface Messages {
        void show(String text);
    }

    private final MapView view;
    private final Messages messages;
    private MapGroup planGroup;
    private MapGroup beliefGroup;

    public SearchOverlay(MapView view, Messages messages) {
        this.view = view;
        this.messages = messages;
    }

    @Override
    public void clear() {
        MapGroup g = group(false);
        if (g != null)
            g.clearItems();
    }

    /** Removes the plan and the belief layers. */
    public void clearAll() {
        clear();
        MapGroup b = group(true);
        if (b != null)
            b.clearItems();
    }

    @Override
    public void drawPlan(SearchPlan plan, boolean preview) {
        MapGroup g = group(false);
        if (g == null)
            return;
        if (plan.prior != null)
            drawTopCells(plan.prior, g, HEAT_RGB);
        for (int i = 0; i < plan.routes.size(); i++) {
            DroneRoute r = plan.routes.get(i);
            if (r.waypoints.size() < 2)
                continue;
            GeoPoint[] pts = new GeoPoint[r.waypoints.size()];
            for (int k = 0; k < pts.length; k++) {
                Waypoint w = r.waypoints.get(k);
                pts[k] = new GeoPoint(w.lat, w.lon);
            }
            Polyline line = new Polyline(UUID.randomUUID().toString());
            line.setPoints(pts);
            line.setStrokeColor(ROUTE_COLORS[i % ROUTE_COLORS.length]);
            line.setStrokeWeight(preview ? 2d : 4d);
            if (preview)
                line.setStrokeStyle(Shape.BASIC_LINE_STYLE_DASHED);
            line.setTitle(r.droneId + " search route (" + r.producedBy + ")");
            decorate(line);
            g.addItem(line);
        }
    }

    @Override
    public void drawBelief(ProbabilityGrid where, ProbabilityGrid searched, BeliefSummary summary) {
        MapGroup g = group(true);
        if (g == null)
            return;
        g.clearItems();
        int drawn = 0;
        for (int c = 0; c < searched.values.length && drawn < SEARCHED_CELLS; c++) {
            if (searched.values[c] >= 0.5) {
                g.addItem(square(searched, c, SEARCHED_ARGB));
                drawn++;
            }
        }
        drawTopCells(where, g, HEAT_RGB);
    }

    @Override
    public void message(String text) {
        if (messages != null)
            messages.show(text);
    }

    /** The highest-valued cells as filled squares, more opaque where the value is higher. */
    private static void drawTopCells(ProbabilityGrid g, MapGroup into, int rgb) {
        if (g.values.length == 0)
            return;
        Integer[] order = new Integer[g.values.length];
        for (int i = 0; i < order.length; i++)
            order[i] = i;
        Arrays.sort(order, (x, y) -> Double.compare(g.values[y], g.values[x]));
        double max = g.values[order[0]];
        if (!(max > 0))
            return;
        for (int k = 0; k < Math.min(HEATMAP_CELLS, order.length); k++) {
            int c = order[k];
            double v = g.values[c];
            if (v <= 0)
                break;
            int alpha = (int) (40 + 160 * v / max);
            into.addItem(square(g, c,
                    Color.argb(alpha, Color.red(rgb), Color.green(rgb), Color.blue(rgb))));
        }
    }

    private static Polyline square(ProbabilityGrid g, int c, int argb) {
        LocalPlane pl = new LocalPlane(g.northWest);
        int r = c / g.cols;
        int col = c % g.cols;
        double x0 = col * g.cellSizeM;
        double y0 = -r * g.cellSizeM;
        LatLon a = pl.toLatLon(x0, y0);
        LatLon b = pl.toLatLon(x0 + g.cellSizeM, y0 - g.cellSizeM);
        GeoPoint[] sq = {new GeoPoint(a.lat, a.lon), new GeoPoint(a.lat, b.lon),
                new GeoPoint(b.lat, b.lon), new GeoPoint(b.lat, a.lon)};
        Polyline cell = new Polyline(UUID.randomUUID().toString());
        cell.setPoints(sq);
        cell.setStyle(cell.getStyle() | Polyline.STYLE_CLOSED_MASK | Shape.STYLE_FILLED_MASK);
        cell.setFillColor(argb);
        cell.setStrokeColor(Color.TRANSPARENT);
        cell.setStrokeWeight(1d);
        decorate(cell);
        return cell;
    }

    /** Local display items: not sent as CoT, not listed, not editable. */
    private static void decorate(Shape s) {
        s.setClickable(false);
        s.setMetaBoolean("nevercot", true);
        s.setMetaBoolean("addToObjList", false);
        s.setMetaBoolean("editable", false);
    }

    private MapGroup group(boolean belief) {
        if (view == null)
            return null;
        if (belief) {
            if (beliefGroup == null)
                beliefGroup = findOrAdd(GROUP_BELIEF);
            return beliefGroup;
        }
        if (planGroup == null)
            planGroup = findOrAdd(GROUP_PLAN);
        return planGroup;
    }

    private MapGroup findOrAdd(String name) {
        MapGroup root = view.getRootGroup();
        MapGroup g = root.findMapGroup(name);
        return g != null ? g : root.addGroup(name);
    }
}
