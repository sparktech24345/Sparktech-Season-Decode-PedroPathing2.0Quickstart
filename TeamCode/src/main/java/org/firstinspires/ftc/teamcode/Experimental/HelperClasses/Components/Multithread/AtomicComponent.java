package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Multithread;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;

import java.util.HashMap;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public abstract class AtomicComponent {

    protected AtomicReference<HashMap<String, Double>> states = new AtomicReference<>();
    protected AtomicReference<Double> target = new AtomicReference<>(0.0);
    protected AtomicReference<Double> resolution = new AtomicReference<>(1.0);
    protected AtomicReference<Double> min_range = new AtomicReference<>(1.0);
    protected AtomicReference<Double> max_range = new AtomicReference<>(1.0);
    protected AtomicBoolean moveOnInit = new AtomicBoolean(false);

    public <T extends AtomicComponent> T addState(String s, double v) {
        states.get().put(s, v);
        return (T) this;
    }

    public <T extends AtomicComponent> T addState(String s, double v, boolean setAsDefault) {
        states.get().put(s, v);
        target.set(v);
        return (T) this;
    }

    public <T extends AtomicComponent> T setRange(double min, double max) {
        min_range.set(min);
        max_range.set(max);
        return (T) this;
    }

    public <T extends AtomicComponent> T loadState(String s) {
        target.set(states.get().get(s));
        return (T) this;
    }

    public boolean hasStateOfName(String state_name) {
        if (!states.get().containsKey(state_name)) return false;
        return target.get().equals(states.get().get(state_name));
    }

    public abstract double getPosition();

    public <T extends AtomicComponent> T setResolution(double res) {
        resolution.set(res);
        return (T) this;
    }

    public boolean moveDuringInit() {
        return moveOnInit.get();
    }

    public <T extends AtomicComponent> T moveDuringInit(boolean move) {
        moveOnInit.set(move);
        return (T) this;
    }

    public abstract void update();
}
