package com.atakmap.android.plugintemplate.plugin.mavlink;

/** ArduCopter custom_mode numbers. */
public final class CopterMode {
    public static final int STABILIZE = 0;
    public static final int AUTO = 3;
    public static final int GUIDED = 4;
    public static final int LOITER = 5;
    public static final int RTL = 6;
    public static final int LAND = 9;
    public static final int BRAKE = 17;
    public static final int SMART_RTL = 21;

    private CopterMode() {
    }

    public static String name(long mode) {
        switch ((int) mode) {
            case STABILIZE: return "STABILIZE";
            case 1: return "ACRO";
            case 2: return "ALT_HOLD";
            case AUTO: return "AUTO";
            case GUIDED: return "GUIDED";
            case LOITER: return "LOITER";
            case RTL: return "RTL";
            case 7: return "CIRCLE";
            case LAND: return "LAND";
            case 16: return "POSHOLD";
            case BRAKE: return "BRAKE";
            case SMART_RTL: return "SMART_RTL";
            default: return mode < 0 ? "—" : ("MODE " + mode);
        }
    }
}
