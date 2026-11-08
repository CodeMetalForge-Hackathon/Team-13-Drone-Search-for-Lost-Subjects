package com.atakmap.android.plugintemplate.plugin.search;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.Looper;

import com.atakmap.android.ipc.AtakBroadcast;
import com.mindfulpigeon.atak.bridge.IntentRequestHandler;
import com.mindfulpigeon.search.api.IntentContract;
import com.mindfulpigeon.search.api.SearchPlanner;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

/**
 * The search engine's JSON intent API (search/docs/API.md section 2) for other ATAK components:
 * ACTION_PLAN / ACTION_CANCEL in, ACTION_PLAN_PROGRESS / ACTION_PLAN_RESULT out, all on ATAK's
 * in-process AtakBroadcast bus (not system broadcasts, so plans are not visible to other apps).
 */
public final class PlanIntentReceiver extends BroadcastReceiver {

    private final IntentRequestHandler handler;
    private final File cacheDir;

    public PlanIntentReceiver(Context pluginContext, SearchPlanner planner) {
        Handler main = new Handler(Looper.getMainLooper());
        this.handler = new IntentRequestHandler(planner, main::post);
        this.cacheDir = pluginContext.getCacheDir();
    }

    /** Registers for the plan and cancel actions. */
    public void register() {
        AtakBroadcast.DocumentedIntentFilter f = new AtakBroadcast.DocumentedIntentFilter();
        f.addAction(IntentContract.ACTION_PLAN,
                "Plan a drone search from request JSON (P.U.L.S.E. search engine)");
        f.addAction(IntentContract.ACTION_CANCEL, "Cancel a running search plan");
        AtakBroadcast.getInstance().registerReceiver(this, f);
    }

    public void dispose() {
        AtakBroadcast.getInstance().unregisterReceiver(this);
        handler.close();
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        String id = intent.getStringExtra(IntentContract.EXTRA_REQUEST_ID);
        if (IntentContract.ACTION_CANCEL.equals(intent.getAction())) {
            handler.cancel(id);
            return;
        }
        if (!IntentContract.ACTION_PLAN.equals(intent.getAction()))
            return;
        String json = intent.getStringExtra(IntentContract.EXTRA_REQUEST_JSON);
        long deadline = intent.getLongExtra(IntentContract.EXTRA_DEADLINE_MS, 20_000);
        long seed = intent.getLongExtra(IntentContract.EXTRA_SEED, 1);
        handler.handle(json, id, deadline, seed, new IntentRequestHandler.Reply() {
            @Override
            public void progress(String requestId, String stage, double fraction, double best) {
                Intent p = new Intent(IntentContract.ACTION_PLAN_PROGRESS);
                p.putExtra(IntentContract.EXTRA_REQUEST_ID, requestId);
                p.putExtra(IntentContract.EXTRA_STAGE, stage);
                p.putExtra(IntentContract.EXTRA_FRACTION, fraction);
                p.putExtra(IntentContract.EXTRA_BEST, best);
                AtakBroadcast.getInstance().sendBroadcast(p);
            }

            @Override
            public void result(String requestId, String planJson) {
                Intent r = new Intent(IntentContract.ACTION_PLAN_RESULT);
                r.putExtra(IntentContract.EXTRA_REQUEST_ID, requestId);
                byte[] bytes = planJson.getBytes(StandardCharsets.UTF_8);
                if (bytes.length > IntentContract.MAX_INLINE_JSON_BYTES) {
                    File f = new File(cacheDir, "plan-" + requestId + ".json");
                    try (FileOutputStream out = new FileOutputStream(f)) {
                        out.write(bytes);
                        r.putExtra(IntentContract.EXTRA_PLAN_FILE, f.getAbsolutePath());
                    } catch (IOException e) {
                        r.putExtra(IntentContract.EXTRA_ERROR,
                                "could not write plan file: " + e.getMessage());
                    }
                } else {
                    r.putExtra(IntentContract.EXTRA_PLAN_JSON, planJson);
                }
                AtakBroadcast.getInstance().sendBroadcast(r);
            }

            @Override
            public void error(String requestId, String message) {
                Intent r = new Intent(IntentContract.ACTION_PLAN_RESULT);
                r.putExtra(IntentContract.EXTRA_REQUEST_ID, requestId);
                r.putExtra(IntentContract.EXTRA_ERROR, message);
                AtakBroadcast.getInstance().sendBroadcast(r);
            }
        });
    }
}
