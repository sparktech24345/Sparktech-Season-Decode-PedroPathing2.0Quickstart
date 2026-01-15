package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.util.function.BooleanSupplier;

public abstract class Action {

    protected boolean start = false;
    protected boolean done = false;
    protected String name = "ACTION_NAME";

    protected BooleanSupplier StartCondition;
    protected BooleanSupplier DoneCondition;
    protected Runnable Execution;
    protected Runnable OnStart;
    protected Runnable OnDone;

    public Action() {
        StartCondition = () -> true;
        DoneCondition = () -> true;
        Execution = () -> {};
        OnStart = () -> {};
        OnDone = () -> {};
    }

    public Action setName(String name) {
        this.name = name;
        return this;
    }

    public String getName() {
        return name;
    }

    public boolean finished() {
        return done;
    }

    public boolean started() {
        return start;
    }

    public void telemetry() {
        MultipleTelemetry tel = RobotController.telemetry;
        tel.addData("at action", name);
        tel.addData("started", start);
        tel.addData("finished", done);
        tel.addData("name", name);
    }

    public Action setStartCondition(BooleanSupplier start) {
        this.StartCondition = start;
        return this;
    }

    public Action setDoneCondition(BooleanSupplier done) {
        this.DoneCondition = done;
        return this;
    }

    public Action setOnStart(Runnable onStart) {
        this.OnStart = onStart;
        return this;
    }

    public Action setOnDone(Runnable onDone) {
        this.OnDone = onDone;
        return this;
    }

    public void update() {
        if (!start) {
            start = StartCondition.getAsBoolean();
            if (start) OnStart.run();
        }
        if (!done && start) {
            Execution.run();
            done = DoneCondition.getAsBoolean();
            if (done) OnDone.run();
        }
    }
}
