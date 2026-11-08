package com.atakmap.android.plugintemplate.plugin.ui;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;

/**
 * The outline of a shape the operator is drawing on the map ({lat, lon} corners, in outline
 * order). Every tap is a corner, in any order and as many as wanted, and the outline never crosses
 * itself. A new corner follows the previous tap when it can without a crossing, so corners tapped
 * around the area keep their order (a concave L stays an L); otherwise it goes between the two
 * corners where it widens (or notches) the area least without a crossing, so corners tapped out of
 * order still make one area. Three corners make an area. A tap on an existing corner (a double
 * tap) adds nothing; Undo takes back the last tap.
 */
final class ShapeDraft {

    static final int MIN_CORNERS = 3;
    /** Taps closer than this to a corner are that corner, metres. */
    private static final double SAME_SPOT_M = 0.5;
    private static final double M_PER_DEG = 111_320;

    private List<double[]> ring = new ArrayList<>();
    /** The last tap, which the next one follows when it can. */
    private double[] last;
    /** The outline and last tap before each tap, for Undo. */
    private final Deque<Object[]> history = new ArrayDeque<>();

    /** @return false when the tap is on an existing corner */
    boolean add(double lat, double lon) {
        double[] p = {lat, lon};
        for (double[] c : ring)
            if (metres(c, p) < SAME_SPOT_M)
                return false;
        history.push(new Object[] {copy(ring), last});
        ring = insert(ring, indexOf(ring, last), p);
        last = p.clone();
        return true;
    }

    /** Takes back the last tap. @return false when there was none */
    @SuppressWarnings("unchecked")
    boolean undo() {
        if (history.isEmpty())
            return false;
        Object[] before = history.pop();
        ring = (List<double[]>) before[0];
        last = (double[]) before[1];
        return true;
    }

    private static int indexOf(List<double[]> ring, double[] p) {
        for (int i = 0; p != null && i < ring.size(); i++)
            if (ring.get(i)[0] == p[0] && ring.get(i)[1] == p[1])
                return i;
        return ring.size() - 1;
    }

    int size() {
        return ring.size();
    }

    boolean canFinish() {
        return ring.size() >= MIN_CORNERS;
    }

    List<double[]> corners() {
        return copy(ring);
    }

    // ---- geometry (a local flat plane in metres around the first corner) --------------

    /**
     * The outline with p added: right after corner {@code after} (the previous tap) if that
     * crosses no side; else between the two neighbouring corners where it adds the least length
     * and crosses no side. If no side takes it without a crossing, the corners are ordered by
     * bearing around their centre, which never crosses.
     */
    private static List<double[]> insert(List<double[]> ring, int after, double[] p) {
        List<double[]> out = copy(ring);
        int n = ring.size();
        if (n < MIN_CORNERS) {
            out.add(after + 1, p);
            return out;
        }
        Plane pl = new Plane(ring.get(0));
        double[][] xy = new double[n][];
        for (int i = 0; i < n; i++)
            xy[i] = pl.xy(ring.get(i));
        double[] q = pl.xy(p);
        if (fitsBetween(xy, after, q)) {
            out.add(after + 1, p);
            return out;
        }
        int best = -1;
        double bestCost = Double.POSITIVE_INFINITY;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double cost = dist(xy[i], q) + dist(q, xy[j]) - dist(xy[i], xy[j]);
            if (cost < bestCost && fitsBetween(xy, i, q)) {
                bestCost = cost;
                best = i;
            }
        }
        if (best >= 0) {
            out.add(best + 1, p);
            return out;
        }
        out.add(p);
        return byBearing(out);
    }

    /** Whether the sides i-q and q-(i+1) cross no other side of the outline. */
    private static boolean fitsBetween(double[][] xy, int i, double[] q) {
        int n = xy.length;
        int j = (i + 1) % n;
        for (int k = 0; k < n; k++) {
            if (k == i)
                continue; // the side q replaces
            int k2 = (k + 1) % n;
            // a side meeting a new side at a shared corner does not count as a crossing
            if (k2 != i && segmentsMeet(xy[i], q, xy[k], xy[k2]))
                return false;
            if (k != j && segmentsMeet(q, xy[j], xy[k], xy[k2]))
                return false;
        }
        return true;
    }

    private static List<double[]> byBearing(List<double[]> pts) {
        double lat = 0, lon = 0;
        for (double[] p : pts) {
            lat += p[0];
            lon += p[1];
        }
        final double[] c = {lat / pts.size(), lon / pts.size()};
        final Plane pl = new Plane(c);
        List<double[]> out = copy(pts);
        Collections.sort(out, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                double[] pa = pl.xy(a), pb = pl.xy(b);
                return Double.compare(Math.atan2(pa[1], pa[0]), Math.atan2(pb[1], pb[0]));
            }
        });
        return out;
    }

    /** Whether no two sides of the closed outline cross or touch (other than neighbours at their shared corner). */
    static boolean isSimple(List<double[]> ring) {
        int n = ring.size();
        if (n < MIN_CORNERS)
            return true;
        Plane pl = new Plane(ring.get(0));
        double[][] xy = new double[n][];
        for (int i = 0; i < n; i++)
            xy[i] = pl.xy(ring.get(i));
        for (int a = 0; a < n; a++) {
            for (int b = a + 1; b < n; b++) {
                boolean neighbours = b == a + 1 || (a == 0 && b == n - 1);
                if (!neighbours && segmentsMeet(xy[a], xy[(a + 1) % n], xy[b], xy[(b + 1) % n]))
                    return false;
            }
        }
        return true;
    }

    /** Whether segments ab and cd cross or touch. */
    private static boolean segmentsMeet(double[] a, double[] b, double[] c, double[] d) {
        double d1 = orient(c, d, a), d2 = orient(c, d, b), d3 = orient(a, b, c), d4 = orient(a, b, d);
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
            return true;
        return (d1 == 0 && onSegment(c, d, a)) || (d2 == 0 && onSegment(c, d, b))
                || (d3 == 0 && onSegment(a, b, c)) || (d4 == 0 && onSegment(a, b, d));
    }

    private static double orient(double[] a, double[] b, double[] c) {
        double v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        return Math.abs(v) < 1e-9 ? 0 : v; // square metres: below a pin's width is collinear
    }

    private static boolean onSegment(double[] a, double[] b, double[] p) {
        return Math.min(a[0], b[0]) - 1e-9 <= p[0] && p[0] <= Math.max(a[0], b[0]) + 1e-9
                && Math.min(a[1], b[1]) - 1e-9 <= p[1] && p[1] <= Math.max(a[1], b[1]) + 1e-9;
    }

    private static double dist(double[] a, double[] b) {
        return Math.hypot(a[0] - b[0], a[1] - b[1]);
    }

    private static double metres(double[] a, double[] b) {
        return dist(new Plane(a).xy(a), new Plane(a).xy(b));
    }

    private static List<double[]> copy(List<double[]> pts) {
        List<double[]> out = new ArrayList<>(pts.size());
        for (double[] p : pts)
            out.add(p.clone());
        return out;
    }

    /** East and north metres from an origin ({lat, lon} in, {x, y} out). */
    private static final class Plane {
        private final double lat0, lon0, cos;

        Plane(double[] origin) {
            lat0 = origin[0];
            lon0 = origin[1];
            cos = Math.cos(Math.toRadians(lat0));
        }

        double[] xy(double[] p) {
            return new double[] {(p[1] - lon0) * M_PER_DEG * cos, (p[0] - lat0) * M_PER_DEG};
        }
    }
}
