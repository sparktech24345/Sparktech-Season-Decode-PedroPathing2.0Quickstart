package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import com.pedropathing.geometry.*;

import java.util.function.BooleanSupplier;

public class MoveAction extends Action {

    public MoveAction(Pose moveTargetPos) {
        super();
        this.OnStart = () -> {
            follower.interrupt();
            follower.follow(moveTargetPos);
        };
        this.DoneCondition = () -> /*start &&*/ !follower.isMoving();
    }
}
