package com.atakmap.android.plugintemplate.plugin.search;

import com.atakmap.android.importexport.handlers.ParentMapItem;
import com.atakmap.android.maps.MapGroup;
import com.atakmap.android.maps.MapItem;
import com.atakmap.android.maps.MapView;
import com.atakmap.coremap.maps.coords.GeoPoint;
import com.mindfulpigeon.atak.bridge.TakMapAccess;
import com.mindfulpigeon.search.geo.LatLon;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Reads closed shapes and point markers from the TAK map for the search engine (titles carry the
 * naming conventions: SEARCH, RZ, LKP, BOOST, LAUNCH, RECOVER, CLUE). The plugin's own overlay
 * groups are skipped.
 *
 * <p>When the MAVLink drone has a GPS fix and the map has no LAUNCH marker, the drone's position is
 * offered as the marker "LAUNCH 1 (drone)", so drone-1's route starts where the drone is.
 *
 * <p>ATAK's Shape and Marker are written out in full: the TakMapAccess interface's own nested
 * Shape and Marker types shadow imports inside this class.
 */
public final class AtakMapAccess implements TakMapAccess {

    /** The connected drone's position, or null when unknown. */
    public interface DronePosition {
        LatLon get();
    }

    public static final String DRONE_LAUNCH_UID = "koalified-search-drone-launch";

    private final MapView view;
    private final DronePosition drone;
    private final String launchPrefix;
    private final Set<String> skipGroups = new HashSet<>();

    public AtakMapAccess(MapView view, DronePosition drone, String launchPrefix,
            String... skipGroupNames) {
        this.view = view;
        this.drone = drone;
        this.launchPrefix = launchPrefix.toUpperCase(Locale.ROOT);
        for (String g : skipGroupNames)
            skipGroups.add(g);
    }

    @Override
    public List<TakMapAccess.Shape> shapes() {
        List<TakMapAccess.Shape> out = new ArrayList<>();
        if (view != null)
            collect(view.getRootGroup(), out, null, partGroups());
        return out;
    }

    @Override
    public List<TakMapAccess.Marker> markers() {
        List<TakMapAccess.Marker> out = new ArrayList<>();
        if (view != null)
            collect(view.getRootGroup(), null, out, partGroups());
        LatLon d = drone != null ? drone.get() : null;
        if (d != null) {
            boolean hasLaunch = false;
            for (TakMapAccess.Marker m : out) {
                if (m.title.trim().toUpperCase(Locale.ROOT).startsWith(launchPrefix)) {
                    hasLaunch = true;
                    break;
                }
            }
            if (!hasLaunch)
                out.add(new TakMapAccess.Marker(DRONE_LAUNCH_UID, launchPrefix + " 1 (drone)", d));
        }
        return out;
    }

    @Override
    public LatLon self() {
        if (view == null)
            return null;
        com.atakmap.android.maps.Marker self = view.getSelfMarker();
        GeoPoint p = self != null ? self.getPoint() : null;
        if (p == null || !p.isValid() || (p.getLatitude() == 0 && p.getLongitude() == 0))
            return null;
        return LatLon.of(p.getLatitude(), p.getLongitude());
    }

    /**
     * The groups holding the parts of a drawn item (a circle's rings, a shape's corner and centre
     * markers). Parts carry their item's title, so reading them would count the item twice: a
     * circle titled RZ 1 would be two red zones, and the centre marker of a shape titled LKP ...
     * would be an LKP.
     */
    private Set<MapGroup> partGroups() {
        Set<MapGroup> parts = new HashSet<>();
        findParts(view.getRootGroup(), parts);
        return parts;
    }

    private static void findParts(MapGroup g, Set<MapGroup> parts) {
        if (g == null)
            return;
        for (MapItem item : g.getItems()) {
            if (item instanceof ParentMapItem) {
                MapGroup child = ((ParentMapItem) item).getChildMapGroup();
                if (child != null)
                    parts.add(child);
            }
        }
        Collection<MapGroup> children = g.getChildGroups();
        if (children != null)
            for (MapGroup child : children)
                findParts(child, parts);
    }

    private void collect(MapGroup g, List<TakMapAccess.Shape> shapes,
            List<TakMapAccess.Marker> markers, Set<MapGroup> parts) {
        if (g == null || skipGroups.contains(g.getFriendlyName()) || parts.contains(g))
            return;
        for (MapItem item : g.getItems()) {
            String title = item.getTitle();
            if (shapes != null && item instanceof com.atakmap.android.maps.Shape) {
                GeoPoint[] pts = ((com.atakmap.android.maps.Shape) item).getPoints();
                if (pts != null && pts.length >= 3) {
                    List<LatLon> ring = new ArrayList<>(pts.length);
                    for (GeoPoint p : pts)
                        ring.add(LatLon.of(p.getLatitude(), p.getLongitude()));
                    shapes.add(new TakMapAccess.Shape(item.getUID(), title, ring));
                }
            } else if (markers != null && item instanceof com.atakmap.android.maps.Marker) {
                GeoPoint p = ((com.atakmap.android.maps.Marker) item).getPoint();
                if (p != null && p.isValid())
                    markers.add(new TakMapAccess.Marker(item.getUID(), title,
                            LatLon.of(p.getLatitude(), p.getLongitude())));
            }
        }
        Collection<MapGroup> children = g.getChildGroups();
        if (children != null)
            for (MapGroup child : children)
                collect(child, shapes, markers, parts);
    }
}
