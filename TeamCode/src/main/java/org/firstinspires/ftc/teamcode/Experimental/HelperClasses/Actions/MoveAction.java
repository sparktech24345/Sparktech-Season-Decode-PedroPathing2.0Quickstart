package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;

public class MoveAction extends Action {

    public MoveAction(Pose moveTargetPos) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos);
        this.DoneCondition = () -> ComplexFollower.done();
    }
    public MoveAction(Pose moveTargetPos,Pose bezierHelper,boolean shouldReverse,boolean shouldHoldHEading) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos,bezierHelper,shouldReverse,shouldHoldHEading);
        this.DoneCondition = () -> ComplexFollower.done();
    }
    public MoveAction(Pose moveTargetPos,boolean shouldImproviseBezierOnX,boolean shouldReverse,boolean shouldHoldHEading) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos,shouldImproviseBezierOnX,shouldReverse,shouldHoldHEading);
        this.DoneCondition = () -> ComplexFollower.done();
    }
}
