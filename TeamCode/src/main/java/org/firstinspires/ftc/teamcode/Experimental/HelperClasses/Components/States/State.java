package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorizedComponent;

public class State {

    private MotorizedComponent<?> owner = null;
    private final double value;

    public State(double value) {
        this.value = value;
    }

    public void own(MotorizedComponent<?> c) {
        if (owner != null) return;
        owner = c;
    }

    public double value() {
        return value;
    }

    public MotorizedComponent<?> owner() {
        return owner;
    }
}