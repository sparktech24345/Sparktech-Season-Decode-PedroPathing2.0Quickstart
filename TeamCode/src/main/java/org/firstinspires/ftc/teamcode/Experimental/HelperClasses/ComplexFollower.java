package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.ITDConstants.ITDcreateFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.Drawing.DashboardDrawing;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.ITDConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.DashboardPoseTracker;

import java.util.ArrayDeque;
import java.util.Queue;

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
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    private DashboardDrawing dashboardDrawing;
    private final Queue<Pose> poseQueue = new ArrayDeque<>();


    ComplexFollower(HardwareMap hardwareMap) {
        this.follower = ITDcreateFollower(hardwareMap);
        follower.update();
        currentPos = startPose;
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();

        poseUpdater = new PoseUpdater(hardwareMap,ITDConstants.class);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        dashboardDrawing = new DashboardDrawing(follower,dashboardPoseTracker);
    }

    public void follow(Pose targetPos) {
        poseQueue.add(targetPos);
    }

    public void Continue() {
        shouldContinue = true;
    }

    public Follower getInstance() { return follower; }

    public boolean done() { return isDone; }

    public void update() {
        if (currentOpModes == OpModes.Autonomous) {
            follower.update();
            dashboardDrawing.update();
            if (follower.isBusy()) {
                isDone = false;
                currentPos = follower.getPose();
                currentX = currentPos.getX();
                currentY = currentPos.getY();
                currentHeading = currentPos.getHeading();
            }
            else if (shouldContinue && !poseQueue.isEmpty()) {
                currentTargetPos = poseQueue.poll();
                pathToFollow = new Path(new BezierLine(currentPos, currentTargetPos));
                pathToFollow.setLinearHeadingInterpolation(currentPos.getHeading(),currentTargetPos.getHeading());
                follower.followPath(pathToFollow);
                shouldContinue = false;
                follower.update();
            }
            else isDone = true;
        }
    }

    public void stopAll() {
        interrupt();
        poseQueue.clear();
    }

    public void interrupt() {
        if (follower.isBusy()) follower.breakFollowing();
    }

    public void telemetry() {
        telemetryInstance.addData("Follower X", currentX);
        telemetryInstance.addData("Follower Y", currentY);
        telemetryInstance.addData("Follower Heading", currentHeading);
        telemetryInstance.addData("Follower is busy", follower.isBusy());
    }
}
