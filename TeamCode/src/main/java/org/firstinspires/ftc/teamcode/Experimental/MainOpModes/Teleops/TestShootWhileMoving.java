package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;
@Disabled
@TeleOp(name = "Test shooting")
public class TestShootWhileMoving extends LinearOpMode {
    private static Pose copy_pose(Pose source) {
        Pose a = new Pose(source.getX(), source.getY(), source.getHeading());
        return a;
    }

    private Pose target_pos = new Pose(10, 20);

    private static final double shoot_time_close_min_angle_ms = 670;
    private static final double shoot_time_close_max_angle_ms = 690;
    private static final double shoot_time_far_min_angle_ms   = 1670;
    private static final double shoot_time_far_max_angle_ms   = 1690;

    @Override
    public void runOpMode() {
        //init

        while (opModeInInit()) {
            //init loop
        }
        //start

        while(opModeIsActive()) {
            //loop
        }
        //stop
    }
}
