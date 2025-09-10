package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.pedropathing.geometry.*;

import java.util.function.BooleanSupplier;

public class MoveAction extends Action {

    private boolean reachTarget = true;
    private final Pose moveTargetPos;

    public MoveAction(boolean waitForPrevious, Pose moveTargetPos) {
        super(waitForPrevious);
        this.moveTargetPos = moveTargetPos;

        this.Execution = () -> {
            followerInstance.follow(moveTargetPos);
            followerInstance.Continue();
        };
    }

    public MoveAction(boolean waitForPrevious, String posName) {
        super(waitForPrevious);
        this.moveTargetPos = robotControllerInstance.getAutoPose(posName);

        this.Execution = () -> {
            followerInstance.follow(moveTargetPos);
            followerInstance.Continue();
        };
    }

    public MoveAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }
}
