package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;

public abstract class MotorizedComponent<T extends MotorizedComponent<T>> {

    protected double last_target = 0;
    protected double target = 0;
    protected double resolution = 1;
    protected double min_range = 1;
    protected double max_range = -1;

    public T self() {
        return (T) this;
    }

    public T setRange(double min, double max) {
        min_range = min;
        max_range = max;
        return self();
    }

    public final T setState(State state) {
        if (state.owner() != this || state.value() == target) return self();
        target = state.value();
        this.update();
        return self();
    }

    public T setState(double value) {
        target = value;
        return self();
    }

    public T setResolution(double res) {
        resolution = res;
        return self();
    }

    public abstract void update();

    public void telemetry() {}
}
