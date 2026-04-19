package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

public class TurretAngleStates extends StateSet<ServoComponent<TurretAngleStates>> {

    public final State DEFAULT  = state(180);
    public final State DOWN_MAX = state(316);
    public final State UP_MAX   = state( 18);

    public TurretAngleStates() {
        super();
        def = DEFAULT;
    }
}
