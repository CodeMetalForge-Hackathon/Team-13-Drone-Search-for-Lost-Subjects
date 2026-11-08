package com.atakmap.android.plugintemplate.plugin.terrain;

import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Locale;

/**
 * Copies DTED tiles shipped in the plugin APK ({@code assets/dted/<lonDir>/<tile>.dt?})
 * into ATAK's DTED directory, so terrain works with zero setup and ATAK's own
 * elevation tools see the same data.
 */
public final class DtedAutoloader {

    private static final String TAG = "KoalifiedDted";
    private static final String ASSET_ROOT = "dted";

    private DtedAutoloader() {
    }

    /** @return number of tiles copied or refreshed. */
    public static int install(AssetManager assets, File destRoot) {
        int copied = 0;
        String[] lonDirs;
        try {
            lonDirs = assets.list(ASSET_ROOT);
        } catch (IOException e) {
            Log.w(TAG, "no bundled DTED: " + e.getMessage());
            return 0;
        }
        if (lonDirs == null)
            return 0;

        for (String dir : lonDirs) {
            String[] tiles;
            try {
                tiles = assets.list(ASSET_ROOT + "/" + dir);
            } catch (IOException e) {
                continue;
            }
            if (tiles == null)
                continue;
            for (String tile : tiles) {
                if (!isDtedFile(tile))
                    continue;
                String assetPath = ASSET_ROOT + "/" + dir + "/" + tile;
                File dest = new File(new File(destRoot, dir.toLowerCase(Locale.US)),
                        tile.toLowerCase(Locale.US));
                try {
                    if (upToDate(assets, assetPath, dest))
                        continue;
                    copy(assets, assetPath, dest);
                    copied++;
                } catch (IOException e) {
                    Log.w(TAG, "failed to install " + assetPath + ": " + e.getMessage());
                }
            }
        }
        Log.i(TAG, "bundled DTED installed/refreshed: " + copied + " tile(s) → " + destRoot);
        return copied;
    }

    static boolean isDtedFile(String name) {
        String n = name.toLowerCase(Locale.US);
        return n.endsWith(".dt0") || n.endsWith(".dt1") || n.endsWith(".dt2");
    }

    private static boolean upToDate(AssetManager assets, String assetPath, File dest) {
        if (!dest.isFile() || dest.length() == 0)
            return false;
        try (AssetFileDescriptor fd = assets.openFd(assetPath)) {
            return fd.getLength() == dest.length();
        } catch (IOException e) {
            // compressed asset: cannot get length cheaply; trust the existing file
            return true;
        }
    }

    private static void copy(AssetManager assets, String assetPath, File dest)
            throws IOException {
        File parent = dest.getParentFile();
        if (parent != null && !parent.isDirectory() && !parent.mkdirs())
            throw new IOException("cannot create " + parent);
        File tmp = new File(dest.getPath() + ".part");
        try (InputStream in = assets.open(assetPath);
                OutputStream out = new FileOutputStream(tmp)) {
            byte[] buf = new byte[64 * 1024];
            int n;
            while ((n = in.read(buf)) > 0)
                out.write(buf, 0, n);
        }
        if (dest.exists() && !dest.delete())
            throw new IOException("cannot replace " + dest);
        if (!tmp.renameTo(dest))
            throw new IOException("rename failed for " + dest);
    }
}
