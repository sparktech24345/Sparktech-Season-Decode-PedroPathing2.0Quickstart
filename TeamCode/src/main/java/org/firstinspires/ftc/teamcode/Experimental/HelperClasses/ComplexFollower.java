package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.*;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.text.MessageFormat;
import java.util.List;
import java.util.function.Function;

public class ComplexFollower {

    private static boolean isDone = true;
    private static Pose startPos = pose(0, 0, 0);
    private static Pose lastTargetPose = startPos;
    private static double currentX = startPos.getX();
    private static double currentY = startPos.getY();
    private static double currentHeading = startPos.getHeading();

    private static Pose currentTargetPos = startPos;
    private static Pose currentPos = startPos;
    private static Path pathToFollow;

    private static Follower follower;
    private static HardwareMap hmap;

    private static ElapsedTime follow_timer;

    private static boolean init_ = false;

    public static void setHardwareMap(HardwareMap map) {
        hmap = map;
    }

    /**
     *
     * @param follower_build_function The build function to use for the follower. (Ex: Constants::createFollower)
     *                                  The function must take a HardwareMap and return a Follower.
     */
    public static void init(Function<HardwareMap, Follower> follower_build_function) {
        init(follower_build_function, startPos);
    }

    public static void init(Function<HardwareMap, Follower> follower_build_function, Pose startingPose) {
        if (init_) return;
        follower = follower_build_function.apply(hmap);
       // follower = new Follower(hardwareMap, FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        //com.pedropathing.util.Constants.setConstants(FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        follower.setStartingPose(startingPose);
        lastTargetPose = startingPose;
        follower.update();
        currentPos = startPos;
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
        currentTargetPos = startPos;
        Drawing.drawDebug(follower);
        follower.activateAllPIDFs();
    }

    public static void unInit() {
        init_ = false;
        reset();
    }

    public static boolean isInit() {
        return init_;
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
    public static void hold(Pose holdPose) {
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();

        follower.holdPoint(holdPose,false);
    }
    public static void hold() {
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();

        follower.holdPoint(currentPos,false);
    }
    public static void follow(Pose targetPos) {
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();
        currentTargetPos = targetPos;

        Pose toUsePose;
        //if(lastTargetPose.distanceFrom(currentPos) > 5) toUsePose = currentPos;
         //toUsePose = lastTargetPose;
        toUsePose = currentPos;

        pathToFollow = new Path(new BezierLine(toUsePose, currentTargetPos));
        pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(),currentTargetPos.getHeading());
        follower.followPath(pathToFollow,true);
        isDone = false;

        lastTargetPose = targetPos;
    }
    public static void follow(Pose targetPos, PathConstraints littleConstraints) {
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();
        currentTargetPos = targetPos;

        Pose toUsePose;
        //if(lastTargetPose.distanceFrom(currentPos) > 5) toUsePose = currentPos;
        //toUsePose = lastTargetPose;
        toUsePose = currentPos;

        pathToFollow = new Path(new BezierLine(toUsePose, currentTargetPos),littleConstraints);
        pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(),currentTargetPos.getHeading());
        follower.followPath(pathToFollow,true);
        isDone = false;

        lastTargetPose = targetPos;
    }
    public static void follow(Pose targetPos,BezierCurveTypes bezierCurveType,double headingIfNeeded, Pose... bezierPointHelper){
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();

        Pose toUsePose;
//        if(lastTargetPose.distanceFrom(currentPos) > 5) toUsePose = currentPos;
//        else toUsePose = lastTargetPose;
        toUsePose = currentPos;



        pathToFollow = new Path(new BezierCurve(combinePoseArrays(toUsePose,bezierPointHelper,targetPos)));

        switch(bezierCurveType){
            case TangentHeading:
                pathToFollow.setTangentHeadingInterpolation();
                break;
            case LinearHeading:
                pathToFollow.setLinearHeadingInterpolation(toUsePose.getHeading(),targetPos.getHeading());
                break;
            case ConstantHeading:
                pathToFollow.setConstantHeadingInterpolation(headingIfNeeded);
                break;
            case ReverseTangentHeading:
                pathToFollow.setTangentHeadingInterpolation();
                pathToFollow.reverseHeadingInterpolation();
                break;
            default:
                pathToFollow.setConstantHeadingInterpolation(0);
        }

        follower.followPath(pathToFollow,true);
        isDone = false;

        lastTargetPose = targetPos;
    }
    public static void follow(Pose targetPos, boolean shouldImproviseOnX,BezierCurveTypes bezierCurveType,double headingIfNeeded){
        if (follower == null) return;
        if (follow_timer == null) follow_timer = new ElapsedTime();
        else follow_timer.reset();

        Pose toUsePose;
//        if(lastTargetPose.distanceFrom(currentPos) > 5) toUsePose = currentPos;
//        else toUsePose = lastTargetPose;
        toUsePose = currentPos;


        pathToFollow = new Path(new BezierCurve(toUsePose,improviseBezierCurvePath(toUsePose,targetPos,shouldImproviseOnX),targetPos));

        switch(bezierCurveType){
            case TangentHeading:
                pathToFollow.setTangentHeadingInterpolation();
                break;
            case LinearHeading:
                pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(),targetPos.getHeading());
                break;
            case ConstantHeading:
                pathToFollow.setConstantHeadingInterpolation(headingIfNeeded);
                break;
            case ReverseTangentHeading:
                pathToFollow.setTangentHeadingInterpolation();
                pathToFollow.reverseHeadingInterpolation();
                break;
            default:
                pathToFollow.setConstantHeadingInterpolation(0);
        }

        follower.followPath(pathToFollow,true);
        isDone = false;

        lastTargetPose = targetPos;
    }
    public static Pose improviseBezierCurvePath(Pose currentPose, Pose targetPose, boolean shouldImproviseBezierOnX){
        Pose helperPoint;
        if(shouldImproviseBezierOnX) helperPoint = new Pose(targetPose.getX(),currentPose.getY()); // currents Pose y
        else helperPoint = new Pose(currentPose.getX(),targetPose.getY()); // currents Pose x

        return helperPoint;
    }
    public static Pose[] combinePoseArrays(Pose start, Pose[] middle, Pose end) {
        // 1. Define the exact size needed
        Pose[] combined = new Pose[middle.length + 2];

        // 2. Place the 'start' at the very beginning
        combined[0] = start;

        // 3. High-speed memory copy of the middle array
        // (Source, SourcePos, Dest, DestPos, Length)
        System.arraycopy(middle, 0, combined, 1, middle.length);

        // 4. Place the 'end' at the very last index
        combined[combined.length - 1] = end;

        return combined;
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
    public static double getFollowingTimeMilisec(){
        if(follow_timer == null) return 0;
        return follow_timer.milliseconds();
    }
    public static boolean isDoneFollowingTimer(double timeToPass){
        return getFollowingTimeMilisec() >= timeToPass;
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
        globalRobotPose = currentPos;
    }

    public static void interrupt() {
        if (follower == null) return;
        if (follower.isBusy()) follower.breakFollowing();
    }

    public static void telemetry() {
        if (follower == null) {
            ComplexOpMode.publicTelemetry.addData("[ERROR] Follower", "Follower instance is null!");
            return;
        }
        ComplexOpMode.publicTelemetry.addData("Follower is busy", follower.isBusy());
        ComplexOpMode.publicTelemetry.addData("Current pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentX, currentY, Math.toDegrees(currentHeading)));
        ComplexOpMode.publicTelemetry.addData("Target pose", MessageFormat.format("x: {0} -- y: {1} -- heading: {2}", currentTargetPos.getX(), currentTargetPos.getY(), Math.toDegrees(currentTargetPos.getHeading())));
        ComplexOpMode.publicTelemetry.addData("Absolute Angle",Math.toDegrees(follower.getTotalHeading()));
        ComplexOpMode.publicTelemetry.addData("Follower velocity", follower.getVelocity().getMagnitude());
        ComplexOpMode.publicTelemetry.addData("Is done?", isDone);
    }
}
