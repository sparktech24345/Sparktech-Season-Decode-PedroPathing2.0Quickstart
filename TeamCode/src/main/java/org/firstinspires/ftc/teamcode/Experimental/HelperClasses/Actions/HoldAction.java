package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

//import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos.SmallTriangleNew.bezierHelper3;
//import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos.SmallTriangleNew.weirdHpCollect;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BezierCurveTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;

public class HoldAction extends Action {
    public HoldAction(Pose holdTargetPos,double timeToHoldPose) {
        super();
        this.OnStart = () -> ComplexFollower.hold(holdTargetPos);
        this.DoneCondition = () -> ComplexFollower.isDoneFollowingTimer(timeToHoldPose);
        this.OnDone = () -> {ComplexFollower.interrupt(); ComplexFollower.setStopHolding(false);};
    }
    public HoldAction(double timeToHoldPose) {
        super();
        this.OnStart = () -> ComplexFollower.hold(true);
        this.DoneCondition = () -> ComplexFollower.isDoneFollowingTimer(timeToHoldPose);
        this.OnDone = () -> {ComplexFollower.interrupt(); ComplexFollower.setStopHolding(false);};
    }
//    public HoldAction(int thisIsFuckedUp,double timeToHoldPose) {
//        super();
//
//        if( GlobalStorage.futureMoveActionTargetPose == null) GlobalStorage.futureMoveActionTargetPose = new Pose();
//
//        this.OnStart = checkTarget;
//        this.DoneCondition = () -> ComplexFollower.isDoneFollowingTimer(timeToHoldPose);
//        this.OnDone = () -> {ComplexFollower.interrupt(); ComplexFollower.setStopHolding(false);};
//    }
//    public Runnable checkTarget = () -> {
//        if(GlobalStorage.futureMoveActionTargetPose == weirdHpCollect) ComplexFollower.follow(weirdHpCollect, BezierCurveTypes.LinearHeading,bezierHelper3.getHeading(), bezierHelper3);
//        else ComplexFollower.follow(GlobalStorage.futureMoveActionTargetPose);
//    };
}
