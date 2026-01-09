package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

import java.text.MessageFormat;

public class ComplexFollower {

    private boolean isDone = true;

    private double currentX;
    private double currentY;
    private double currentHeading;

    private Pose currentTargetPos;
    private Pose currentPos;
    private Path pathToFollow;

    private Follower follower;

    public ComplexFollower(HardwareMap hardwareMap) {
        init(hardwareMap, new Pose(0, 0, 0));
    }

    public ComplexFollower(HardwareMap hardwareMap, Pose startingPose) {
        init(hardwareMap, startingPose);
    }

    private void init(HardwareMap hardwareMap, Pose startingPose) {
        this.follower = ConstantsDecode.createFollowerDecode(hardwareMap);
        follower.setStartingPose(startingPose);
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

    public Follower instance() {
        return follower;
    }
    public Pose getTarget() {
        return currentTargetPos;
    }
    public boolean done() {
        return isDone;
    }
    public boolean isMoving() {
        return follower.isBusy() || follower.getVelocity().getMagnitude() >= 0.01;
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

    public void telemetry() {
        RobotController.telemetry.addData("Follower is busy", follower.isBusy());
        RobotController.telemetry.addData("Current pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentX, currentY, currentHeading));
        RobotController.telemetry.addData("Target pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentTargetPos.getX(), currentTargetPos.getY(), currentTargetPos.getHeading()));
        RobotController.telemetry.addData("Follower velocity", follower.getVelocity().getMagnitude());
        RobotController.telemetry.addData("Is done?", isDone);
    }
}
