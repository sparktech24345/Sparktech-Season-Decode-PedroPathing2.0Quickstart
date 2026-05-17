package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class CameraRotateStates extends StateSet<ServoComponent<CameraRotateStates>> {

    public final State ZERO = state(0);

    public CameraRotateStates() {
        super();
        def = ZERO;
    }
}