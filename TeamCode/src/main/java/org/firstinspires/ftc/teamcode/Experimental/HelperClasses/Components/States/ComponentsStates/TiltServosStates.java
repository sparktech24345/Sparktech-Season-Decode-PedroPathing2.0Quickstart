package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class TiltServosStates extends StateSet<ServoComponent<TiltServosStates>> {

    public final State RETRACTED = state(360);
    public final State EXTENDED  = state(198);

    public TiltServosStates() {
        super();
        def = RETRACTED;
    }
}
