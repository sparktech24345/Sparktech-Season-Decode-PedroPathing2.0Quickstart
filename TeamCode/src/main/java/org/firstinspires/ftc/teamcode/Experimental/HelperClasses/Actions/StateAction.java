package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import java.util.function.BooleanSupplier;

public class StateAction extends Action {

    public StateAction(String ComponentName, String PositionName) {
        this.Execution = () -> robotControllerInstance.getComponent(ComponentName).loadState(PositionName);
    }
}
