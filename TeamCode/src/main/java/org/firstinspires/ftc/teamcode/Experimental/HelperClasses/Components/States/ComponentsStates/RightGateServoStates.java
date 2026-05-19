package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class RightGateServoStates extends StateSet<ServoComponent<RightGateServoStates>> {

    public final State CLOSED = state(244.8);
    public final State OPEN   = state(151);

    public RightGateServoStates() {
        super();
        def = CLOSED;
    }
}
