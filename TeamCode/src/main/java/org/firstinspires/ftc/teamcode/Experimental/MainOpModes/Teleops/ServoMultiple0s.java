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
    public static double rightGatePos = 0.21;
    public static double leftGatePos = 0.36;
    public static double angleServoPos = 0.8;
    public static double intakeServoPos = 0.3;
    public static double motorPow = 0;

    @Override
    public void runOpMode() {
        Servo servo0 = hardwareMap.get(Servo.class, rightGateServoName); // 0.5 is open 0.21 is closed
        Servo servo1 = hardwareMap.get(Servo.class, leftGateServoName); // 0.1 is open 0.36 is closed
        Servo servo2 = hardwareMap.get(Servo.class, turretAngleServoName); // 0.9 is max down and 0.73 is max up
        Servo servo3 = hardwareMap.get(Servo.class, intakeSorterServoName); // 0.17 is redirect to right 0.3 is block // 0.44 is redirect to left
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, intakeMotorName);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeInInit()) {
            if (servo0 != null) servo0.setPosition(rightGatePos);
            if (servo1 != null) servo1.setPosition(leftGatePos);
            if (servo2 != null) servo2.setPosition(angleServoPos);
            if (servo3 != null) servo3.setPosition(intakeServoPos);
        }

        while (opModeIsActive()) {
            if (servo0 != null) servo0.setPosition(rightGatePos);
            if (servo1 != null) servo1.setPosition(leftGatePos);
            if (servo2 != null) servo2.setPosition(angleServoPos);
            if (servo3 != null) servo3.setPosition(intakeServoPos);

            motor.setPower(motorPow);
        }
    }
}
