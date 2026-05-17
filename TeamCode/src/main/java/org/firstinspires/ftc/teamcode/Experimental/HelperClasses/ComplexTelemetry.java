package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ComplexTelemetry {
    private static MultipleTelemetry tel = new MultipleTelemetry();
    private static boolean init_ = false;
    private static boolean enabled = true;

    public static boolean isInit() {
        return init_;
    }

    public static void init(Telemetry tele) {
        addTelemetry(tele);
    }

    public static void addTelemetry(Telemetry tele) {
        tel.addTelemetry(tele);
        init_ = true;
    }

    public static void reset() {
        init_ = false;
        tel = new MultipleTelemetry();
    }

    public static MultipleTelemetry get() {
        if (!enabled) return new MultipleTelemetry();
        return tel;
    }

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
    }

    public static boolean isEnabled() {
        return enabled;
    }
}
