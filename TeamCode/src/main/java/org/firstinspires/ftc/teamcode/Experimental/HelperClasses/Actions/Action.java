package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexTelemetry;

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
        ComplexTelemetry.get().addData("started", start);
        ComplexTelemetry.get().addData("finished", done);
        ComplexTelemetry.get().addData("name", name);
    }

    public Action startIf(BooleanSupplier start) {
        this.StartCondition = start;
        return this;
    }

    public Action finishIf(BooleanSupplier done) {
        this.DoneCondition = done;
        return this;
    }

    public Action onStart(Runnable onStart) {
        this.OnStart = onStart;
        return this;
    }

    public Action onFinish(Runnable onDone) {
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
