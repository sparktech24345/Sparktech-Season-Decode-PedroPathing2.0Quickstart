package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp(name = "servoSet0", group = "Linear OpMode")
public class servoSet0 extends LinearOpMode {
    public static double servoPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo1 = hardwareMap.get(Servo.class, "servo1");



        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            if(gamepad1.a) servoPos +=0.0002;
            if(gamepad1.b) servoPos -=0.0002;
            if(gamepad1.y) servoPos = 0;
            if(gamepad1.x) servoPos = 30/360;
            servo1.setPosition(servoPos);
            tel.addData("position",servoPos);
            tel.addData("position * 360",servoPos*360);

            tel.update();
        }
    }


}
