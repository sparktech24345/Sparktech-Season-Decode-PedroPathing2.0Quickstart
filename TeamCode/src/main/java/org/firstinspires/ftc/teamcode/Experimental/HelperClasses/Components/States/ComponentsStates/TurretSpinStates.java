package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class TurretSpinStates extends StateSet<MotorComponent<TurretSpinStates>> {

    public final State OFF  = state(0);
    public final State FULL = state(1);

    public TurretSpinStates() {
        super();
        def = OFF;
    }
}
