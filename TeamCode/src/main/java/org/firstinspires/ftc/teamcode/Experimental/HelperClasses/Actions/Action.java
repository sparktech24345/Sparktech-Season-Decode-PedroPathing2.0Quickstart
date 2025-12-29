package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import java.util.function.BooleanSupplier;

public abstract class Action {

    protected boolean start = false;
    protected boolean done = false;

    protected BooleanSupplier ExecutionCondition = () -> true;
    protected BooleanSupplier DoneCondition = () -> true;
    protected Runnable Execution = () -> {};
    protected Runnable OnStart = () -> {};
    protected Runnable OnDone = () -> {};

    public boolean finished() {
        return DoneCondition.getAsBoolean();
    }

    public boolean started() {
        return start;
    }

    public Action setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }

    public Action setDoneCondition(BooleanSupplier done) {
        this.DoneCondition = done;
        return this;
    }

    public Action setOnStart(Runnable onStart) {
        OnStart = onStart;
        return this;
    }

    public Action setOnDone(Runnable onDone) {
        OnDone = onDone;
        return this;
    }

    public void update() {
        if (!start) {
            start = ExecutionCondition.getAsBoolean();
            if (start) OnStart.run();
        }
        if (!done && start) {
            Execution.run();
            done = DoneCondition.getAsBoolean();
            if (done) OnDone.run();
        }
    }
}
