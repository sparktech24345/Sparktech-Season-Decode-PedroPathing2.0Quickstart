package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public abstract class Component {

    protected double last_target = 0;
    protected double target = 0;
    protected double resolution = 1;
    protected double min_range = 1;
    protected double max_range = -1;


    protected abstract void update();

    public abstract void telemetry();
}
