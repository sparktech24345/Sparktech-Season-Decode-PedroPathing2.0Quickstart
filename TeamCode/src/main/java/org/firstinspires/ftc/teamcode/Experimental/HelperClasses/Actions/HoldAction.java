package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;

public class HoldAction extends Action {
    public HoldAction(Pose holdTargetPos,double timeToHoldPose) {
        super();
        this.OnStart = () -> ComplexFollower.hold(holdTargetPos);
        this.DoneCondition = () -> ComplexFollower.isDoneFollowingTimer(timeToHoldPose);
        this.OnDone = () -> ComplexFollower.interrupt();
    }
}
