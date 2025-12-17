package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

import java.lang.Math;


@Disabled
@TeleOp(name = "AutoAimTurret", group = "LinearOpMode")
public class AutoAimTurret extends LinearOpMode {
    Pose target = new Pose(5, 5, 0);
    double turretServoPos = 0;
    Follower follower;

    double angleToTargetDeg(double px, double py, double tx, double ty) {
        return Math.toDegrees(Math.atan2(ty - py, tx - px));
    }

    double shortestSignedAngleDeg(double targetAngle, double heading) {
        double diff = (targetAngle - heading + 180.0) % 360.0 - 180.0;
        // Java's % can be negative, so fix:
        if (diff <= -180.0) diff += 360.0;
        if (diff > 180.0) diff -= 360.0;
        return diff;
    }

    double getServoPos(Pose current, Pose target) {
        double targetAngle = Math.toDegrees(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
        double heading = Math.toDegrees(current.getHeading());
        double diff = (targetAngle - heading + 180.0) % 360.0 - 180.0;
        // Java's % can be negative, so fix:
        if (diff <= -180.0) diff += 360.0;
        if (diff > 180.0) diff -= 360.0;
        return diff;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretServo = hardwareMap.get(Servo.class, "turretServo");
        follower = ConstantsDecode.createFollowerDecode(hardwareMap);

        while (opModeIsActive()) {
            follower.update();
            double result1, result2;
            result1 = getServoPos(follower.getPose(), target);
            /* OR */
            result2 = shortestSignedAngleDeg(
                    angleToTargetDeg(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            target.getX(), target.getY()
                    ),
                    Math.toDegrees(follower.getHeading())
            );

            turretServoPos = result2;
            turretServo.setPosition(turretServoPos);

            if (result1 != result2) telemetry.addData("Result", "same");
            else telemetry.addData("Result", "different");
            telemetry.update();
        }
    }
}