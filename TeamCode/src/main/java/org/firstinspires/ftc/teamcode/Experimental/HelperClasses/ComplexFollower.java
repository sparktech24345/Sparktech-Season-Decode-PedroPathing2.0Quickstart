package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

import java.text.MessageFormat;

public class ComplexFollower {

    private static boolean isDone = true;

    private static double currentX;
    private static double currentY;
    private static double currentHeading;

    private static Pose currentTargetPos;
    private static Pose currentPos;
    private static Path pathToFollow;

    private static Follower follower;

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, pose(0, 0, 0));
    }

    public static void init(HardwareMap hardwareMap, Pose startingPose) {
        follower = ConstantsDecode.createFollowerDecode(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        currentPos = startPose;
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
        currentTargetPos = startPose;
    }

    public static void setStartingPose(Pose start) {
        if (follower == null) return;
        follower.setStartingPose(start);
    }

    public static void follow(Pose targetPos) {
        if (follower == null) return;
        follow(targetPos, null);
    }

    public static void follow(Pose targetPos, PathConstraints pathConstraints) {
        if (follower == null) return;
        currentTargetPos = targetPos;
        pathToFollow = new Path(new BezierLine(currentPos, currentTargetPos));
        pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(), currentTargetPos.getHeading());
        if (pathConstraints != null) pathToFollow.setConstraints(pathConstraints);
        follower.followPath(pathToFollow);
    }

    public static Follower instance() {
        return follower;
    }
    public static Pose getTarget() {
        if (follower == null) return pose(0, 0, 0);
        return currentTargetPos;
    }
    public static boolean done() {
        if (follower == null) return true;
        return isDone;
    }
    public static boolean isMoving() {
        if (follower == null) return false;
        return follower.isBusy() || follower.getVelocity().getMagnitude() >= 0.01;
    }

    public static void update() {
        if (follower == null) return;
        follower.update();
        isDone = !follower.isBusy();
        currentPos = follower.getPose();
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
    }

    public static void interrupt() {
        if (follower == null) return;
        if (follower.isBusy()) follower.breakFollowing();
    }

    public static void telemetry() {
        if (follower == null) {
            RobotController.telemetry.addData("[ERROR] Follower", "Follower instance is null!");
            return;
        }
        RobotController.telemetry.addData("Follower is busy", follower.isBusy());
        RobotController.telemetry.addData("Current pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentX, currentY, currentHeading));
        RobotController.telemetry.addData("Target pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentTargetPos.getX(), currentTargetPos.getY(), currentTargetPos.getHeading()));
        RobotController.telemetry.addData("Follower velocity", follower.getVelocity().getMagnitude());
        RobotController.telemetry.addData("Is done?", isDone);
    }
}
