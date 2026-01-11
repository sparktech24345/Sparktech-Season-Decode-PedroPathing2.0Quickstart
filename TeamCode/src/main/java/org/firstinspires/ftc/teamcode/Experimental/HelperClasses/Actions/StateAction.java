package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

public class StateAction extends Action {

    public StateAction(String ComponentName, String PositionName) {
        super();
        this.Execution = () -> robotController.getComponent(ComponentName).loadState(PositionName);
    }

    public StateAction(String ComponentName, double TargetPosition) {
        super();
        this.Execution = () -> robotController.getComponent(ComponentName).setTarget(TargetPosition);
    }
}