package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

public class StateAction extends Action {

    public StateAction(String ComponentName, String PositionName) {
        super();
        this.Execution = () -> robotControllerInstance.getComponent(ComponentName).loadState(PositionName);
    }

    public StateAction(String ComponentName, double TargetPosition) {
        super();
        this.Execution = () -> robotControllerInstance.getComponent(ComponentName).setTarget(TargetPosition);
    }
}