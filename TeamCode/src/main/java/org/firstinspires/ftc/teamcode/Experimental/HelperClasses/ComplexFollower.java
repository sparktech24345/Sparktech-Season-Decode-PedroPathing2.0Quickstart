package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.*;
import com.pedropathing.pathgen.*;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.FConstantsForPinpoint;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.LConstantsForPinpoint;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

import java.text.MessageFormat;

public class ComplexFollower {

    private static boolean isDone = true;
    private static Pose startPos = pose(0, 0, 0);

    private static double currentX = startPos.getX();
    private static double currentY = startPos.getY();
    private static double currentHeading = startPos.getHeading();

    private static Pose currentTargetPos = startPos;
    private static Pose currentPos = startPos;
    private static Path pathToFollow;

    private static Follower follower;
    private static HardwareMap hmap;

    private static ElapsedTime follow_timer;

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, startPos);
    }

    public static void init(HardwareMap hardwareMap, Pose startingPose) {
        hmap = hardwareMap;
        follower = new Follower(hardwareMap, FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        com.pedropathing.util.Constants.setConstants(FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        follower.setStartingPose(startingPose);
        follower.update();
        currentPos = startPos;
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
        currentTargetPos = startPos;
    }

    public static void reset() {
        isDone = true;
        currentX = startPos.getX();
        currentY = startPos.getY();
        currentHeading = startPos.getHeading();
        currentTargetPos = startPos;
        currentPos = startPos;
        pathToFollow = null;
        follower = null;
    }

    public static void resetAndInit() {
        reset();
        if (hmap == null) return;
        init(hmap);
    }

    public static double followingForMS() {
        if (follow_timer == null) return 0;
        return follow_timer.milliseconds();
    }

    public static void setStartingPose(Pose start) {
        startPos = start;
        if (follower == null) return;
        follower.setStartingPose(start);
    }

    public static Pose getCurrentPose() {
        return currentPos;
    }

    public static void setPose(Pose pose) {
        if (follower == null) return;
        follower.setPose(pose);
    }

    public static void follow(Pose targetPos) {
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();
        currentTargetPos = targetPos;
        pathToFollow = new Path(new BezierLine(currentPos, currentTargetPos));
        pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(), currentTargetPos.getHeading());
        follower.followPath(pathToFollow,true);
        isDone = false;
    }
    public static void follow(Pose targetPos, Pose bezierPointHelper, boolean shouldReverse,boolean holdHeading){
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();

        Point pointCurrent = poseToPoint(currentPos);
        Point bezierExtraPoint = poseToPoint(bezierPointHelper);
        Point pointTarget = poseToPoint(targetPos);

        pathToFollow = new Path(new BezierCurve(pointCurrent,bezierExtraPoint,pointTarget));
        if(holdHeading) pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(), targetPos.getHeading());
        pathToFollow.setReversed(shouldReverse);
        follower.followPath(pathToFollow,true);
        isDone = false;
    }
    public static void follow(Pose targetPos, boolean shouldImproviseBezierOnX, boolean shouldReverse, boolean holdHeading){
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();

        pathToFollow = improviseBezierCurvePath(currentPos, targetPos,shouldImproviseBezierOnX);
        if(holdHeading) pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(), targetPos.getHeading());
        pathToFollow.setReversed(shouldReverse);
        follower.followPath(pathToFollow,true);
        isDone = false;
    }


    /// Bezier improvising stuff \\\
    public static Point poseToPoint(Pose pose){
        return new Point(pose.getX(),pose.getY(),Point.CARTESIAN);
    }
    public static Path improviseBezierCurvePath(Pose currentPose, Pose targetPose, boolean shouldImproviseBezierOnX){
        Point helperPoint;
        if(shouldImproviseBezierOnX) helperPoint = new Point(targetPose.getX(),currentPose.getY(),Point.CARTESIAN); // currents Pose y
        else helperPoint = new Point(currentPose.getX(),targetPose.getY(),Point.CARTESIAN); // currents Pose x

        return new Path(new BezierCurve(
                poseToPoint(currentPose),
                helperPoint,
                poseToPoint(targetPose)
        ));
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
        return follower.isBusy()
        ;
    }

    public static void update() {
        if (follower == null) return;
        follower.update();
        isDone = !follower.isBusy();
        currentPos = follower.getPose();
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
        Drawing.drawDebug(follower);
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
        RobotController.telemetry.addData("Current pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentX, currentY, Math.toDegrees(currentHeading)));
        RobotController.telemetry.addData("Target pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentTargetPos.getX(), currentTargetPos.getY(), Math.toDegrees(currentTargetPos.getHeading())));
        RobotController.telemetry.addData("Absolute Angle",Math.toDegrees(follower.getTotalHeading()));
        RobotController.telemetry.addData("Follower velocity", follower.getVelocity().getMagnitude());
        RobotController.telemetry.addData("Is done?", isDone);
    }
}
