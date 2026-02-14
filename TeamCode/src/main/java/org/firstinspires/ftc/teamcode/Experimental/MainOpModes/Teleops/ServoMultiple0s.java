package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "ServoMultiple0", group = "Linear OpMode")
public class ServoMultiple0s extends LinearOpMode {
    public static double rightGatePos = 0;
    public static double leftGatePos = 0;
    public static double leftTiltPos = 1;//  poses are syncronized, in bot = 1, out of bot = 0
    public static double rightTiltPos = 1;
    public static double angleServoPos = 0.4; // max down is 0.88 and max up is 0.05
    public static double motorPow = 0;

    @Override
    public void runOpMode() {
        Servo leftTiltServo = hardwareMap.get(Servo.class, leftTiltServoName);
        Servo rightTiltServo = hardwareMap.get(Servo.class, rightTiltServoName);

        Servo rightGateServo = hardwareMap.get(Servo.class, rightGateServoName);
        Servo leftGateServo = hardwareMap.get(Servo.class, leftGateServoName);

        Servo turretAngleServo = hardwareMap.get(Servo.class, turretAngleServoName);


        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeInInit()) {
            if (leftTiltServo != null) leftTiltServo.setPosition(leftTiltPos);// poses are syncronized, in bot = 1, out of bot = 0
            if (rightTiltServo != null) rightTiltServo.setPosition(rightTiltPos);

            if (rightGateServo != null) rightGateServo.setPosition(rightGatePos);
            if (leftGateServo != null) leftGateServo.setPosition(leftGatePos);

            if (turretAngleServo != null) turretAngleServo.setPosition(angleServoPos); // max down is 0.88 and max up is 0.05
        }

        while (opModeIsActive()) {
            if (leftTiltServo != null) leftTiltServo.setPosition(leftTiltPos);
            if (rightTiltServo != null) rightTiltServo.setPosition(rightTiltPos);

            if (rightGateServo != null) rightGateServo.setPosition(rightGatePos);
            if (leftGateServo != null) leftGateServo.setPosition(leftGatePos);

            if (turretAngleServo != null) turretAngleServo.setPosition(angleServoPos);
            motor.setPower(motorPow);
        }
    }
}
