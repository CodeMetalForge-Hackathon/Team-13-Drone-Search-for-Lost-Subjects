package com.atakmap.android.plugintemplate.plugin.terrain;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

/**
 * Fetches DTED tiles on demand from a DTED.org-style stream endpoint
 * ({@code <base>/w118/n34.dt2.zip}) into ATAK's DTED directory, best level first
 * (dt2 → dt1 → dt0). ATAK's own Dt2FileWatcher rescans that directory every
 * few seconds, so downloaded tiles are used by ATAK as well as by the plugin.
 */
public class DtedDownloader {

    public static final String DEFAULT_BASE_URL = "https://ca.dted.org";

    /** Levels best-first. */
    static final String[] LEVELS = {"dt2", "dt1", "dt0"};

    public interface Fetcher {
        /** @return the response stream, or null if the tile does not exist (404). */
        InputStream open(String url) throws IOException;
    }

    public interface Progress {
        void onStatus(String message);
    }

    private final String baseUrl;
    private final File destRoot;
    private final Fetcher fetcher;
    /** "w072/n42.dt2" paths known to be absent on the server this session. */
    private final Set<String> missing = Collections.synchronizedSet(new HashSet<>());

    public DtedDownloader(String baseUrl, File destRoot) {
        this(baseUrl, destRoot, DtedDownloader::httpOpen);
    }

    DtedDownloader(String baseUrl, File destRoot, Fetcher fetcher) {
        this.baseUrl = baseUrl.endsWith("/") ? baseUrl.substring(0, baseUrl.length() - 1)
                : baseUrl;
        this.destRoot = destRoot;
        this.fetcher = fetcher;
    }

    /** 1° cells (SW-corner keys like "w118/n34") covering the given points. */
    public static Set<String> cellsFor(Collection<double[]> latLons) {
        Set<String> cells = new LinkedHashSet<>();
        for (double[] p : latLons) {
            String rel = BundledDtedSource.relativePath(p[0], p[1], "x");
            cells.add(rel.substring(0, rel.length() - 2));
        }
        return cells;
    }

    /**
     * Make sure each cell has the best level the server offers.
     * @return number of tiles downloaded
     */
    public int ensureCells(Collection<String> cells, Progress progress)
            throws InterruptedException {
        int n = 0;
        for (String cell : cells) {
            if (Thread.interrupted())
                throw new InterruptedException();
            if (ensureCell(cell, progress))
                n++;
        }
        return n;
    }

    /** @return true if a tile was downloaded */
    boolean ensureCell(String cell, Progress progress) {
        for (String level : LEVELS) {
            String rel = cell + "." + level;
            if (localTile(rel) != null)
                return false; // already have this level, and nothing better is offered
            if (missing.contains(rel))
                continue;
            String url = baseUrl + "/" + rel + ".zip";
            try (InputStream in = fetcher.open(url)) {
                if (in == null) {
                    missing.add(rel);
                    continue;
                }
                if (progress != null)
                    progress.onStatus("Downloading terrain " + rel + " …");
                install(in, rel);
                if (progress != null)
                    progress.onStatus("Installed terrain " + rel);
                return true;
            } catch (IOException e) {
                if (progress != null)
                    progress.onStatus("Terrain " + rel + " failed: " + e.getMessage());
                // network trouble: don't fall back to a coarser level we'd then keep forever
                return false;
            }
        }
        return false;
    }

    private File localTile(String rel) {
        File f = new File(destRoot, rel);
        if (f.isFile() && f.length() > 0)
            return f;
        f = new File(destRoot, rel.toUpperCase(Locale.US));
        return f.isFile() && f.length() > 0 ? f : null;
    }

    /** Unzips the single DTED entry to destRoot/rel atomically, validating the header. */
    void install(InputStream zipStream, String rel) throws IOException {
        File dest = new File(destRoot, rel);
        File parent = dest.getParentFile();
        if (parent != null && !parent.isDirectory() && !parent.mkdirs())
            throw new IOException("cannot create " + parent);
        String ext = rel.substring(rel.lastIndexOf('.') + 1).toLowerCase(Locale.US);
        File tmp = new File(dest.getPath() + ".part");
        boolean found = false;
        try (ZipInputStream zin = new ZipInputStream(zipStream)) {
            ZipEntry e;
            while ((e = zin.getNextEntry()) != null) {
                if (e.isDirectory() || !e.getName().toLowerCase(Locale.US).endsWith("." + ext))
                    continue;
                try (OutputStream out = new FileOutputStream(tmp)) {
                    byte[] buf = new byte[64 * 1024];
                    int n;
                    while ((n = zin.read(buf)) > 0)
                        out.write(buf, 0, n);
                }
                found = true;
                break;
            }
        }
        if (!found) {
            tmp.delete();
            throw new IOException("no ." + ext + " in archive");
        }
        if (!looksLikeDted(tmp)) {
            tmp.delete();
            throw new IOException("downloaded file is not DTED");
        }
        if (dest.exists() && !dest.delete())
            throw new IOException("cannot replace " + dest);
        if (!tmp.renameTo(dest))
            throw new IOException("rename failed for " + dest);
    }

    private static boolean looksLikeDted(File f) throws IOException {
        if (f.length() < DtedReader.HEADER_LEN)
            return false;
        byte[] b = new byte[3];
        try (InputStream in = new java.io.FileInputStream(f)) {
            if (in.read(b) != 3)
                return false;
        }
        return "UHL".equals(new String(b, StandardCharsets.US_ASCII));
    }

    private static InputStream httpOpen(String url) throws IOException {
        HttpURLConnection c = (HttpURLConnection) new URL(url).openConnection();
        c.setConnectTimeout(15_000);
        c.setReadTimeout(60_000);
        c.setRequestProperty("User-Agent", "PULSE-ATAK-plugin");
        int code = c.getResponseCode();
        if (code == HttpURLConnection.HTTP_NOT_FOUND) {
            c.disconnect();
            return null;
        }
        if (code != HttpURLConnection.HTTP_OK) {
            c.disconnect();
            throw new IOException("HTTP " + code);
        }
        return c.getInputStream();
    }
}
