package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class TurretRotateStates extends StateSet<MotorComponent<TurretRotateStates>> {

    public final State ZERO = state(0);

    public TurretRotateStates() {
        super();
        def = ZERO;
    }
}