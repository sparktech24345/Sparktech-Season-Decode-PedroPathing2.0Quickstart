package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Disabled
@Config
@TeleOp(name = "MotorPortTesting", group = "Tests")
public class MotorPortTesting extends LinearOpMode {
    public static double motorCHubPort1Power =0;
    public static double motorCHubPort2Power =0;
    public static double motorCHubPort3Power =0;
    public static double motorExHubPort0Power =0;
    public static double motorExHubPort1Power =0;
    public static double motorTurretSpinPower =0;
    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor motorCHubPort1 = hardwareMap.dcMotor.get("backpurple");
        DcMotor motorCHubPort2 = hardwareMap.dcMotor.get("intakemotor");
        DcMotor motorCHubPort3 = hardwareMap.dcMotor.get("frontpurple");

        DcMotor motorExHubPort0 = hardwareMap.dcMotor.get("frontgreen");
        DcMotor motorExHubPort1 = hardwareMap.dcMotor.get("backgreen");
        DcMotor motorTurretSpin = hardwareMap.dcMotor.get("turretspin");


        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){

            if(gamepad1.aWasReleased() && motorCHubPort1Power == 0) motorCHubPort1Power = 1;  //backpurple
            else if(gamepad1.aWasReleased() && motorCHubPort1Power == 0.5) motorCHubPort1Power = 0;


            if(gamepad1.bWasReleased() && motorCHubPort2Power == 0) motorCHubPort2Power = 1; //intake motor
            else if(gamepad1.bWasReleased() && motorCHubPort2Power == 0.5) motorCHubPort2Power = 0;


            if(gamepad1.yWasReleased() && motorCHubPort3Power == 0) motorCHubPort3Power = 1; //front purple
            else if(gamepad1.yWasReleased() && motorCHubPort3Power == 0.5) motorCHubPort3Power = 0;


            if(gamepad1.xWasReleased() && motorExHubPort0Power == 0) motorExHubPort0Power = 1; //front green
            else if(gamepad1.xWasReleased() && motorExHubPort0Power == 0.5) motorExHubPort0Power = 0;

            if(gamepad1.rightBumperWasReleased() && motorExHubPort1Power == 0) motorExHubPort1Power = 1; //back green
            else if(gamepad1.rightBumperWasReleased() && motorExHubPort1Power == 0.5) motorExHubPort1Power = 0;

            if(gamepad1.leftBumperWasReleased() && motorTurretSpinPower == 0) motorTurretSpinPower = 1;
            else if(gamepad1.leftBumperWasReleased() && motorTurretSpinPower == 0.5) motorTurretSpinPower = 0;


            motorCHubPort1.setPower(motorCHubPort1Power);
            motorCHubPort2.setPower(motorCHubPort2Power);
            motorCHubPort3.setPower(motorCHubPort3Power);

            motorExHubPort0.setPower(motorExHubPort0Power);
            motorExHubPort1.setPower(motorExHubPort1Power);
            motorTurretSpin.setPower(motorTurretSpinPower);



            tel.addData("motorCHubPort1Power",motorCHubPort1Power);
            tel.addData("motorCHubPort2Power",motorCHubPort2Power);
            tel.addData("motorCHubPort3Power",motorCHubPort3Power);
            tel.addData("motorExHubPort0Power",motorExHubPort0Power);
            tel.addData("motorExHubPort1Power",motorExHubPort1Power);
            tel.update();
        }

    }
}


