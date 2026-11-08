package com.atakmap.android.plugintemplate.plugin.terrain;

import java.io.Closeable;
import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;

/**
 * Reads a single DTED level 0/1/2 tile (MIL-PRF-89020B). Heights are MSL (EGM96) metres.
 * Posts are read on demand from disk, so large DTED2 tiles cost no heap.
 */
public final class DtedReader implements Closeable {

    static final int HEADER_LEN = 3428; // UHL (80) + DSI (648) + ACC (2700)
    static final int VOID = -32767;

    private final RandomAccessFile raf;
    private final double originLat;
    private final double originLon;
    private final double latSpacingDeg;
    private final double lonSpacingDeg;
    private final int numCols; // longitude lines
    private final int numRows; // latitude points per line
    private final int recordLen;

    public DtedReader(File file) throws IOException {
        raf = new RandomAccessFile(file, "r");
        try {
            byte[] uhl = new byte[80];
            raf.readFully(uhl);
            String h = new String(uhl, StandardCharsets.US_ASCII);
            if (!h.startsWith("UHL"))
                throw new IOException("not a DTED file: " + file);
            originLon = parseDms(h.substring(4, 12));
            originLat = parseDms(h.substring(12, 20));
            lonSpacingDeg = Integer.parseInt(h.substring(20, 24).trim()) / 36000.0;
            latSpacingDeg = Integer.parseInt(h.substring(24, 28).trim()) / 36000.0;
            numCols = Integer.parseInt(h.substring(47, 51).trim());
            numRows = Integer.parseInt(h.substring(51, 55).trim());
            recordLen = 8 + 2 * numRows + 4;
            if (numCols < 2 || numRows < 2)
                throw new IOException("bad DTED dimensions in " + file);
        } catch (IOException | RuntimeException e) {
            raf.close();
            throw (e instanceof IOException) ? (IOException) e : new IOException(e);
        }
    }

    public double getOriginLat() {
        return originLat;
    }

    public double getOriginLon() {
        return originLon;
    }

    public boolean contains(double lat, double lon) {
        double maxLat = originLat + latSpacingDeg * (numRows - 1);
        double maxLon = originLon + lonSpacingDeg * (numCols - 1);
        return lat >= originLat && lat <= maxLat && lon >= originLon && lon <= maxLon;
    }

    /** Bilinear-interpolated height, or NaN if outside the tile / all void. */
    public synchronized double getElevation(double lat, double lon) throws IOException {
        if (!contains(lat, lon))
            return Double.NaN;
        double c = (lon - originLon) / lonSpacingDeg;
        double r = (lat - originLat) / latSpacingDeg;
        int c0 = Math.min((int) Math.floor(c), numCols - 2);
        int r0 = Math.min((int) Math.floor(r), numRows - 2);
        double fc = c - c0;
        double fr = r - r0;

        int[] west = readPair(c0, r0);
        int[] east = readPair(c0 + 1, r0);
        double[] v = {west[0], east[0], west[1], east[1]};
        double[] w = {(1 - fc) * (1 - fr), fc * (1 - fr), (1 - fc) * fr, fc * fr};

        double sum = 0, wsum = 0;
        for (int i = 0; i < 4; i++) {
            if (v[i] == VOID)
                continue;
            sum += v[i] * w[i];
            wsum += w[i];
        }
        if (wsum <= 0) {
            // all neighbours void or zero-weight: fall back to any valid post
            for (double val : v)
                if (val != VOID)
                    return val;
            return Double.NaN;
        }
        return sum / wsum;
    }

    /** Heights at (col,row) and (col,row+1). */
    private int[] readPair(int col, int row) throws IOException {
        long off = HEADER_LEN + (long) col * recordLen + 8 + 2L * row;
        raf.seek(off);
        byte[] b = new byte[4];
        raf.readFully(b);
        return new int[] {signedMagnitude(b[0], b[1]), signedMagnitude(b[2], b[3])};
    }

    static int signedMagnitude(byte hi, byte lo) {
        int raw = ((hi & 0xFF) << 8) | (lo & 0xFF);
        return (raw & 0x8000) != 0 ? -(raw & 0x7FFF) : raw;
    }

    /** Parse DDDMMSSH (e.g. "0770000W") to signed decimal degrees. */
    static double parseDms(String s) {
        s = s.trim();
        char hemi = Character.toUpperCase(s.charAt(s.length() - 1));
        String digits = s.substring(0, s.length() - 1);
        // tolerate "DDDMMSS.S" variants by taking fixed positions from the right
        int len = digits.length();
        double sec = Double.parseDouble(digits.substring(len - 2));
        double min = Double.parseDouble(digits.substring(len - 4, len - 2));
        double deg = Double.parseDouble(digits.substring(0, len - 4));
        double v = deg + min / 60.0 + sec / 3600.0;
        return (hemi == 'S' || hemi == 'W') ? -v : v;
    }

    @Override
    public synchronized void close() throws IOException {
        raf.close();
    }
}
