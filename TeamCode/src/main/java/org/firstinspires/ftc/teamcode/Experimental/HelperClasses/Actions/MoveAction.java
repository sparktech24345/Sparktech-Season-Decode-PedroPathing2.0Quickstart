package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BezierCurveTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;

public class MoveAction extends Action {

    public MoveAction(Pose moveTargetPos) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos);
        this.DoneCondition = () -> ComplexFollower.done();
    }
    public MoveAction(Pose targetPos, boolean shouldImproviseOnX, BezierCurveTypes bezierCurveType, double headingIfNeeded) {
        super();
        this.OnStart = () -> ComplexFollower.follow(targetPos,shouldImproviseOnX,bezierCurveType,headingIfNeeded);
        this.DoneCondition = () -> ComplexFollower.done();
    }
    public MoveAction(Pose targetPos,BezierCurveTypes bezierCurveType,double headingIfNeeded, Pose... bezierPointHelper) {
        super();
        this.OnStart = () -> ComplexFollower.follow(targetPos,bezierCurveType,headingIfNeeded,bezierPointHelper);
        this.DoneCondition = () -> ComplexFollower.done();
    }
}
