package com.atakmap.android.plugintemplate.plugin;

import android.content.Context;
import android.util.Log;

import com.atak.plugins.impl.PluginContextProvider;
import com.atakmap.android.maps.MapView;
import com.atakmap.android.plugintemplate.plugin.mavlink.MavlinkLink;
import com.atakmap.android.plugintemplate.plugin.mavlink.VehicleCommands;
import com.atakmap.android.plugintemplate.plugin.search.BitmapTileDecoder;
import com.atakmap.android.plugintemplate.plugin.search.PlanIntentReceiver;
import com.atakmap.android.plugintemplate.plugin.search.TerrainElevation;
import com.atakmap.android.plugintemplate.plugin.terrain.AtakElevationSource;
import com.atakmap.android.plugintemplate.plugin.terrain.BundledDtedSource;
import com.atakmap.android.plugintemplate.plugin.terrain.FinestElevationSource;
import com.atakmap.android.plugintemplate.plugin.terrain.DtedAutoloader;
import com.atakmap.android.plugintemplate.plugin.terrain.DtedDownloader;
import com.atakmap.android.plugintemplate.plugin.ui.MainController;
import com.atakmap.android.plugintemplate.plugin.ui.SequenceRunner;
import com.atakmap.android.plugintemplate.plugin.ui.Services;
import com.atakmap.coremap.filesystem.FileSystemUtils;
import com.mindfulpigeon.search.api.SearchPlanner;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import gov.tak.api.plugin.IPlugin;
import gov.tak.api.plugin.IServiceController;
import gov.tak.api.ui.IHostUIService;
import gov.tak.api.ui.Pane;
import gov.tak.api.ui.PaneBuilder;
import gov.tak.api.ui.ToolbarItem;
import gov.tak.api.ui.ToolbarItemAdapter;
import gov.tak.platform.marshal.MarshalManager;

/** P.U.L.S.E.: ArduPilot mission planning + MAVLink control from ATAK. */
public class PluginTemplate implements IPlugin {

    private static final String TAG = "KoalifiedPlanner";

    IServiceController serviceController;
    Context pluginContext;
    IHostUIService uiService;
    ToolbarItem toolbarItem;
    Pane pane;

    private MavlinkLink link;
    private SequenceRunner runner;
    private BundledDtedSource bundledDted;
    private MainController mainController;
    private PlanIntentReceiver searchIntents;

    public PluginTemplate(IServiceController serviceController) {
        this.serviceController = serviceController;
        final PluginContextProvider ctxProvider = serviceController
                .getService(PluginContextProvider.class);
        if (ctxProvider != null) {
            pluginContext = ctxProvider.getPluginContext();
            pluginContext.setTheme(R.style.ATAKPluginTheme);
        }

        uiService = serviceController.getService(IHostUIService.class);

        // well-known identifier so the toolbar can find the button again after it is moved
        toolbarItem = new ToolbarItem.Builder(
                pluginContext.getString(R.string.app_name),
                MarshalManager.marshal(
                        pluginContext.getResources().getDrawable(R.drawable.ic_launcher),
                        android.graphics.drawable.Drawable.class,
                        gov.tak.api.commons.graphics.Bitmap.class))
                .setListener(new ToolbarItemAdapter() {
                    @Override
                    public void onClick(ToolbarItem item) {
                        showPane();
                    }
                }).setIdentifier(pluginContext.getPackageName())
                .build();
    }

    @Override
    public void onStart() {
        link = new MavlinkLink();
        runner = new SequenceRunner();
        try {
            link.start(MavlinkLink.DEFAULT_PORT);
        } catch (Exception e) {
            Log.w(TAG, "could not listen on UDP " + MavlinkLink.DEFAULT_PORT, e);
        }

        // Terrain: DTED shipped in the APK is installed into ATAK's DTED folder and read directly.
        List<File> dtedRoots = new ArrayList<>();
        final File atakDted = FileSystemUtils.getItem("DTED");
        dtedRoots.add(atakDted);
        File[] others = FileSystemUtils.getItems("DTED");
        if (others != null)
            for (File f : others)
                if (!f.equals(atakDted))
                    dtedRoots.add(f);
        bundledDted = new BundledDtedSource(dtedRoots);
        final Context ctx = pluginContext;
        Thread installer = new Thread(() -> DtedAutoloader.install(ctx.getAssets(), atakDted),
                "KoalifiedDtedInstall");
        installer.setDaemon(true);
        installer.start();

        // Search engine JSON intent API for other ATAK components (search/docs/API.md)
        TerrainElevation searchTerrain = new TerrainElevation(terrain());
        searchIntents = new PlanIntentReceiver(pluginContext, new SearchPlanner()
                .withTileDecoder(new BitmapTileDecoder())
                .withElevation(searchTerrain::elevationHae, searchTerrain.name()));
        searchIntents.register();

        if (uiService != null)
            uiService.addToolbarItem(toolbarItem);
    }

    @Override
    public void onStop() {
        if (uiService != null) {
            uiService.removeToolbarItem(toolbarItem);
            if (pane != null)
                uiService.closePane(pane);
        }
        if (mainController != null) {
            mainController.dispose();
            mainController = null;
        }
        pane = null;
        if (searchIntents != null) {
            searchIntents.dispose();
            searchIntents = null;
        }
        if (runner != null)
            runner.shutdown();
        if (link != null)
            link.stop();
        if (bundledDted != null)
            bundledDted.close();
    }

    /**
     * Terrain for the probability map and mission altitudes: local DTED2/DTED1, then whatever
     * elevation ATAK has loaded, then DTED0 (900 m posts, the only level the DTED server has
     * outside California).
     */
    private FinestElevationSource terrain() {
        return new FinestElevationSource(bundledDted, new AtakElevationSource());
    }

    private void showPane() {
        if (pane == null) {
            Services services = new Services(pluginContext, MapView.getMapView(), link,
                    new VehicleCommands(link), runner,
                    terrain(),
                    new DtedDownloader(DtedDownloader.DEFAULT_BASE_URL,
                            FileSystemUtils.getItem("DTED")));
            mainController = new MainController(services);
            pane = new PaneBuilder(mainController.getView())
                    .setMetaValue(Pane.RELATIVE_LOCATION, Pane.Location.Default)
                    .setMetaValue(Pane.PREFERRED_WIDTH_RATIO, 0.5D)
                    .setMetaValue(Pane.PREFERRED_HEIGHT_RATIO, 0.7D)
                    .build();
        }
        if (!uiService.isPaneVisible(pane))
            uiService.showPane(pane, null);
    }
}
