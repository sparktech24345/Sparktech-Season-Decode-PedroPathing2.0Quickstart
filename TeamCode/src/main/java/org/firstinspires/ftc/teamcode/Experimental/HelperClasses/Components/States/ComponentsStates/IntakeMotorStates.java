package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class IntakeMotorStates extends StateSet<MotorComponent<IntakeMotorStates>> {

    public final State OFF          = state(0);
    public final State SLOW         = state(0.5);
    public final State FULL         = state(1);
    public final State FIRING_POWER = state(1);
    public final State FULL_REVERSE = state(-1);
    public final State SLOW_REVERSE = state(-0.5);

    public IntakeMotorStates() {
        super();
        def = OFF;
    }
}
