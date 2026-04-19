package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;

public class HoldAction extends Action {
    public HoldAction(Pose heldPos, double timeToHoldPose) {
        super();
        this.OnStart = () -> ComplexFollower.hold(heldPos);
        this.DoneCondition = () -> ComplexFollower.isDoneFollowingTimer(timeToHoldPose);
        this.OnDone = ComplexFollower::interrupt;
    }

    public HoldAction(double timeToHoldPose) {
        super();
        this.OnStart = ComplexFollower::hold;
        this.DoneCondition = () -> ComplexFollower.isDoneFollowingTimer(timeToHoldPose);
        this.OnDone = ComplexFollower::interrupt;
    }
}
