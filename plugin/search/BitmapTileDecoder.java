package com.atakmap.android.plugintemplate.plugin.search;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.mindfulpigeon.search.layers.TileDecoder;
import com.mindfulpigeon.search.layers.TileImage;

/** Decodes cached map tiles (PNG / JPEG) with BitmapFactory for the engine's raw-tile map input. */
public final class BitmapTileDecoder implements TileDecoder {
    @Override
    public TileImage decode(byte[] data) {
        Bitmap b = BitmapFactory.decodeByteArray(data, 0, data.length);
        if (b == null)
            return null;
        int w = b.getWidth();
        int h = b.getHeight();
        int[] px = new int[w * h];
        b.getPixels(px, 0, w, 0, 0, w, h);
        b.recycle();
        return new TileImage(w, h, px);
    }
}
