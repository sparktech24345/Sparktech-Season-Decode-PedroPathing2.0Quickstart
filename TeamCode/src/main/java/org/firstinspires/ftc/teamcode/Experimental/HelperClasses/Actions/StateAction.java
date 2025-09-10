package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import java.util.function.BooleanSupplier;

public class StateAction extends Action {

    public StateAction(boolean waitForPrevious, String ComponentName, String PositionName) {
        super(waitForPrevious);
        this.Execution = () -> {
            robotControllerInstance.getComponent(ComponentName).loadState(PositionName);
        };
    }

    public StateAction(boolean waitForPrevious, String RobotStateName) {
        super(waitForPrevious);
        this.Execution = () -> {
            robotControllerInstance.loadRobotState(RobotStateName);
        };
    }

    public StateAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }

    public StateAction setDoneCondition(BooleanSupplier done) {
        this.DoneCondition = done;
        return this;
    }
}
