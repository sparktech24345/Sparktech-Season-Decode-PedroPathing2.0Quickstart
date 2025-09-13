package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexGamepad;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotState;
import org.firstinspires.ftc.teamcode.Experimental.StatesAndPositions.ColorSet;
import org.firstinspires.ftc.teamcode.pedroPathing.ITDConstants;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import java.lang.Math;



@TeleOp(name = "AutoAimTurret", group = "LinearOpMode")
public class AutoAimTurret extends LinearOpMode {
    double turretServoPos=0;
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
            shortestSignedAngleDeg(angleToTargetDeg(follower.getPose().getX(),follower.getPose().getY(),targetX,targetY),follower.getHeading());
        }
    }
}