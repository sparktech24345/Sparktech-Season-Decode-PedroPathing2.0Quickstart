package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftGateServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftTiltServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.rightGateServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.rightTiltServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.turretAngleServoName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

@Config
@TeleOp(name = "HoodAndMotors", group = "Linear OpMode")
public class ShootingHoodAndMotors extends LinearOpMode {
    public static double rightGatePos = 0.75;
    public static double leftGatePos = 0.5;
    public static double leftTiltPos = 0.99;
    public static double rightTiltPos = 1;
    public static double angleServoPos = 0.4;
    public static double motorPow = 0;
    public static double motorPowLeft = 0;
    public static double motorPowRight = 0;

    @Override
    public void runOpMode() {

        Servo rightGateServo = hardwareMap.get(Servo.class, rightGateServoName);
        Servo leftGateServo = hardwareMap.get(Servo.class, leftGateServoName);

        Servo turretAngleServo = hardwareMap.get(Servo.class, turretAngleServoName);


        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);

        DcMotorEx turretSpinL = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        DcMotorEx turretSpinR = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");

        MultipleTelemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {

            if (rightGateServo != null) rightGateServo.setPosition(rightGatePos);
            if (leftGateServo != null) leftGateServo.setPosition(leftGatePos);

            if (turretAngleServo != null) turretAngleServo.setPosition(angleServoPos); // max down is 0.88 and max up is 0.05
        }

        while (opModeIsActive()) {
            if (rightGateServo != null) rightGateServo.setPosition(rightGatePos);
            if (leftGateServo != null) leftGateServo.setPosition(leftGatePos);

            if (turretAngleServo != null) turretAngleServo.setPosition(angleServoPos);

            if (turretSpinL != null) turretSpinL.setPower(motorPowLeft);
            if (turretSpinL != null) tel.addData("speed",turretSpinL.getVelocity());
            if (turretSpinR != null) turretSpinR.setPower(motorPowRight);

            tel.update();

            motor.setPower(motorPow);
        }
    }
}
