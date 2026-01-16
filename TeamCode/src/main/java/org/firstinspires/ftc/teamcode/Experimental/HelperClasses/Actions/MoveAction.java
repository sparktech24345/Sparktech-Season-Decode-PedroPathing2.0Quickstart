package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;

public class MoveAction extends Action {

    public MoveAction(Pose moveTargetPos) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos);
        this.DoneCondition = () -> ComplexFollower.done();
    }

    public MoveAction(Pose moveTargetPos, PathConstraints constraints) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos, constraints);
        this.DoneCondition = () -> ComplexFollower.done();
    }
}
