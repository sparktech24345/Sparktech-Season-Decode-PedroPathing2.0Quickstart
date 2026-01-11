package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.text.MessageFormat;

public class DelayAction extends Action {
    private final ElapsedTime timer;
    private final double waitTimeMS;

    public DelayAction(double milliseconds) {
        super();
        this.waitTimeMS = milliseconds;
        timer = new ElapsedTime();
        this.OnStart = timer::reset;
        this.DoneCondition = () -> timer.milliseconds() >= waitTimeMS;
    }
    public void telemetry() {
        MultipleTelemetry tel = RobotController.telemetry;
        tel.addData("at action", name);
        tel.addData("started", start);
        tel.addData("finished", done);
        tel.addData("type of action", "Delay action");
        tel.addData("Timer ms", MessageFormat.format("{0}ms", timer.milliseconds()));
    }
}
