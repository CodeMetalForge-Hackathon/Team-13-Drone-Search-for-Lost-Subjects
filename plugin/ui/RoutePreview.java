package com.atakmap.android.plugintemplate.plugin.ui;

import com.atakmap.android.maps.MapGroup;
import com.atakmap.android.maps.MapItem;
import com.atakmap.android.maps.MapView;
import com.atakmap.android.maps.Marker;
import com.atakmap.android.maps.Polyline;
import com.atakmap.android.plugintemplate.plugin.planning.RoutePlanner;
import com.atakmap.android.plugintemplate.plugin.planning.Waypoint;
import com.atakmap.coremap.maps.coords.GeoPoint;
import com.atakmap.coremap.maps.coords.GeoPointMetaData;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Draws the planned route on the ATAK map. Main thread only. */
class RoutePreview {

    static final String GROUP_NAME = "P.U.L.S.E.";
    private static final int ROUTE_COLOR = 0xFF00E5FF;

    private final MapView mapView;
    private final List<MapItem> items = new ArrayList<>();
    private MapGroup group;

    RoutePreview(MapView mapView) {
        this.mapView = mapView;
    }

    void show(RoutePlanner.Plan plan) {
        clear();
        MapGroup g = group();
        if (g == null)
            return;

        GeoPoint[] pts = new GeoPoint[plan.waypoints.size() + 1];
        pts[0] = new GeoPoint(plan.startLat, plan.startLon);
        for (int i = 0; i < plan.waypoints.size(); i++) {
            Waypoint wp = plan.waypoints.get(i);
            pts[i + 1] = new GeoPoint(wp.lat, wp.lon);
        }
        Polyline line = new Polyline("koalified-route");
        line.setPoints(GeoPointMetaData.wrap(pts));
        line.setStrokeColor(ROUTE_COLOR);
        line.setStrokeWeight(4d);
        line.setTitle("P.U.L.S.E. route");
        line.setClickable(false);
        line.setMetaBoolean("nevercot", true);
        line.setMetaBoolean("addToObjList", false);
        g.addItem(line);
        items.add(line);

        for (int i = 0; i < plan.waypoints.size(); i++) {
            Waypoint wp = plan.waypoints.get(i);
            Marker m = new Marker("koalified-wp-" + (i + 1));
            m.setPoint(new GeoPoint(wp.lat, wp.lon));
            m.setType("b-m-p-w");
            m.setTitle(String.format(Locale.US, "WP%d %.0fm MSL", i + 1, wp.targetAmsl));
            m.setMovable(false);
            m.setMetaBoolean("nevercot", true);
            m.setMetaBoolean("editable", false);
            m.setMetaBoolean("addToObjList", false);
            g.addItem(m);
            items.add(m);
        }
    }

    void clear() {
        if (group != null) {
            for (MapItem i : items)
                group.removeItem(i);
        }
        items.clear();
    }

    private MapGroup group() {
        if (group == null && mapView != null) {
            MapGroup root = mapView.getRootGroup();
            group = root.findMapGroup(GROUP_NAME);
            if (group == null)
                group = root.addGroup(GROUP_NAME);
        }
        return group;
    }
}
