package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorizedComponent;

import java.util.ArrayList;
import java.util.List;

public abstract class StateSet<C extends MotorizedComponent<C>> {
    protected C component;
    protected State def = null;
    protected final List<State> states = new ArrayList<>();

    public StateSet() {}

    protected StateSet(C component) {
        init(component);
    }

    public void init(C component) {
        this.component = component;
        for (State s : states) s.own(component);
    }

    public State defaultState() {
        if (def != null) return def;
        State s = new State(0);
        s.own(component);
        return s;
    }

    protected State state(double value) {
        State s = new State(value);
        if (states.isEmpty()) def = s;
        states.add(s);
        return s;
    }
}