package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "servoSpin", group = "Linear OpMode")
public class servoSpin extends LinearOpMode {
    public static double servoPos = 30/360;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo servo1 = hardwareMap.get(CRServo.class, "turretrotateleft");



        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            if(gamepad1.a) servoPos +=0.0002;
            if(gamepad1.b) servoPos -=0.0002;
            if(gamepad1.y) servoPos = 0;
            if(gamepad1.x) servoPos = (double) 30 / 360;
            servo1.setPower(servoPos);
            tel.addData("position",servoPos);
            tel.addData("position * 360",servoPos*360);

            tel.update();
        }
    }


}
