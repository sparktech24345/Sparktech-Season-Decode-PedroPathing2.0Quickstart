package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class DelayAction extends Action {
    private final ElapsedTime timer;
    private final double waitTimeMS;

    public DelayAction(double milliseconds) {
        this.waitTimeMS = milliseconds;
        timer = new ElapsedTime();
        this.OnStart = timer::reset;
        this.DoneCondition = () -> timer.milliseconds() >= waitTimeMS;
    }

    public DelayAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }
}
