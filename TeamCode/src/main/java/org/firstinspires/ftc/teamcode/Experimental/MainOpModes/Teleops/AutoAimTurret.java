package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.ITDConstants;

import java.lang.Math;



@TeleOp(name = "AutoAimTurret", group = "LinearOpMode")
public class AutoAimTurret extends LinearOpMode {
    double turretServoPos = 0;
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


    double targetX = 5;
    double targetY = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretServo = hardwareMap.get(Servo.class, "turretServo");
        Follower follower = ITDConstants.ITDcreateFollower(hardwareMap);

        while (opModeIsActive()) {
            turretServo.setPosition(turretServoPos);
            follower.update();
            shortestSignedAngleDeg(
                    angleToTargetDeg(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            targetX,
                            targetY
                    ),
                    follower.getHeading()
            );
        }
    }
}