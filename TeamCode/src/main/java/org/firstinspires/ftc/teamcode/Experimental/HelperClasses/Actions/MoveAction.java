package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos.SmallTriangleNew.bezierHelper3;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos.SmallTriangleNew.weirdHpCollect;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BezierCurveTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;

public class MoveAction extends Action {

    public MoveAction(Pose moveTargetPos) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos);
        this.DoneCondition = () -> ComplexFollower.done();
    }
    public MoveAction(Pose moveTargetPos, PathConstraints constraints) {
        super();
        this.OnStart = () -> ComplexFollower.follow(moveTargetPos,constraints);
        this.DoneCondition = () -> ComplexFollower.done();
    }
    public MoveAction(boolean ThisIsAFutureMoveAction){
        super();

        if( GlobalStorage.futureMoveActionTargetPose == null) GlobalStorage.futureMoveActionTargetPose = new Pose();

        this.OnStart = () -> ComplexFollower.follow(GlobalStorage.futureMoveActionTargetPose);
        this.DoneCondition = () -> ComplexFollower.done();
    }
    public MoveAction(int isThisFuckedUpMoveAction){
        super();

        if( GlobalStorage.futureMoveActionTargetPose == null) GlobalStorage.futureMoveActionTargetPose = new Pose();

        this.OnStart = checkTarget;
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
    public Runnable checkTarget = () -> {
        if(GlobalStorage.futureMoveActionTargetPose == weirdHpCollect) ComplexFollower.follow(weirdHpCollect,BezierCurveTypes.ConstantHeading,bezierHelper3.getHeading(), bezierHelper3);
        else ComplexFollower.follow(GlobalStorage.futureMoveActionTargetPose);
    };
}

