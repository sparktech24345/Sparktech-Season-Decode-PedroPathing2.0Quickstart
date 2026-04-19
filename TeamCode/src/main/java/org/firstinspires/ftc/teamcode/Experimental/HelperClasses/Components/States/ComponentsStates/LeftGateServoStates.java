package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class LeftGateServoStates extends StateSet<ServoComponent<LeftGateServoStates>> {

    public final State CLOSED = state(227);
    public final State OPEN   = state(72);

    public LeftGateServoStates() {
        super();
        def = CLOSED;
    }
}
