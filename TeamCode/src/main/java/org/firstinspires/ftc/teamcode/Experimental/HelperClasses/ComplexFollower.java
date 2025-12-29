package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

import java.text.MessageFormat;

public class ComplexFollower {

    private boolean shouldContinue = true;
    private boolean isDone = true;

    private double currentX;
    private double currentY;
    private double currentHeading;

    private Pose currentTargetPos;
    private Pose currentPos;
    private Path pathToFollow;

    private final Follower follower;


    public ComplexFollower(HardwareMap hardwareMap) {
        this.follower = ConstantsDecode.createFollowerDecode(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        currentPos = startPose;
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
        currentTargetPos = startPose;
    }

    public void setStartingPose(Pose start) {
        if (follower == null) return;
        follower.setStartingPose(start);
    }

    public void follow(Pose targetPos) {
        follow(targetPos, null);
    }

    public void follow(Pose targetPos, PathConstraints pathConstraints) {
        currentTargetPos = targetPos;
        pathToFollow = new Path(new BezierLine(currentPos, currentTargetPos));
        pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(), currentTargetPos.getHeading());
        if (pathConstraints != null) pathToFollow.setConstraints(pathConstraints);
        follower.followPath(pathToFollow);
    }

    public Follower instance() { return follower; }
    public Pose getTarget() {
        return currentTargetPos;
    }
    public boolean done() { return isDone; }
    public boolean isMoving() {
        return !isDone || currentPos.minus(currentTargetPos).getAsVector().getMagnitude() >= 0.01 || follower.getVelocity().getMagnitude() >= 0.01;
    }

    public void update() {
        follower.update();
        isDone = !follower.isBusy();
        currentPos = follower.getPose();
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
    }

    public void interrupt() {
        if (follower.isBusy()) follower.breakFollowing();
    }

    public void telemetry(Telemetry tel) {
        tel.addData("Follower is busy:", follower.isBusy());
        tel.addData("Current pose:", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentX, currentY, currentHeading));
        tel.addData("Target pose:", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentTargetPos.getX(), currentTargetPos.getY(), currentTargetPos.getHeading()));
        tel.addData("Should continue?", shouldContinue);
        tel.addData("Is done?", isDone);
    }
}
