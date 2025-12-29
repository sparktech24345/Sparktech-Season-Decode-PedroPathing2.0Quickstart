package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PerformanceProfiler {
    protected ElapsedTime timer = new ElapsedTime();
    protected String name;
    protected DisplayUnit unit;

    public enum DisplayUnit {
        SECONDS,
        MILLISECONDS,
        NANOSECONDS
    }

    public PerformanceProfiler(String name, DisplayUnit unit) {
        this.name = name;
        this.unit = unit;
        timer.reset();
    }

    public void reset() {
        timer.reset();
    }

    public void end() {
        RobotController.telemetry.addData(name, () -> {
            switch(unit) {
                case SECONDS:
                    return timer.seconds();
                case NANOSECONDS:
                    return timer.nanoseconds();
                default:
                    return timer.milliseconds();
            }
        });
    }
}
