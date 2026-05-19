package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public abstract class Component {

    protected abstract void update();

    public abstract void telemetry();
}
