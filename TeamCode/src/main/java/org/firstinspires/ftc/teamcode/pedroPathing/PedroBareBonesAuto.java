package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "PedroBareBonesAuto", group = "tests")
public class PedroBareBonesAuto extends OpMode {
    public static double DISTANCE = 40;
    private int forward = 1;
    long starttime;

    private Path forwards;
    private Path backwards;
    private Path intermediate;

    @Override
    public void init() {
        follower = ConstantsDecode.createFollowerDecode(hardwareMap);
        follower.setStartingPose(new Pose(0, 0));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        follower.update();
        Drawing.drawDebug(follower);
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(0,0), new Pose(DISTANCE + 0,0)));
        forwards.setConstantHeadingInterpolation(0);

        intermediate = new Path(new BezierLine(new Pose(DISTANCE + 0,0), new Pose(DISTANCE + 0,DISTANCE + 0)));
        forwards.setConstantHeadingInterpolation(0);

        backwards = new Path(new BezierLine(new Pose(DISTANCE + 0,DISTANCE +  0), new Pose(0,0)));
        backwards.setConstantHeadingInterpolation(0);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        starttime = System.currentTimeMillis();
        follower.update();

        if (!follower.isBusy()) {
            if (forward == 1) {
                forward++;
                follower.followPath(forwards);
            } else if(forward == 2){
                forward++;
                follower.followPath(intermediate);
            } else{
                forward = 1;
                follower.followPath(backwards);
            }
        }
        Drawing.drawDebug(follower);
        telemetry.addData("timer",starttime - System.currentTimeMillis());
        telemetry.update();
    }
}