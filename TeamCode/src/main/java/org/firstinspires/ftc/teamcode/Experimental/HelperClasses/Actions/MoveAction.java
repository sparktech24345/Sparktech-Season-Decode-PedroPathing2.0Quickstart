package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;

public class MoveAction extends Action {

    public MoveAction(Pose moveTargetPos) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos);
        this.DoneCondition = () -> ComplexFollower.done();
    }
}
