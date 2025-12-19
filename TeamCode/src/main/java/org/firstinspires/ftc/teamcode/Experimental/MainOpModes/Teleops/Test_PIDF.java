package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name="PIDF" ,group="LinearOpMode")
public class Test_PIDF extends LinearOpMode {
    DcMotorEx turretSpin;
    public static double P = 0;
    public static double D = 0;
    public static double I = 0;
    public static double F = 0;
    public static double currentVel;
    public static double targetVel;

    @Override
    public void runOpMode(){
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretspin");
        turretSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretSpin.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients PIDFCoefficients = new PIDFCoefficients(P,I,D,F);
        turretSpin.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            turretSpin.setVelocity(targetVel);
            currentVel = turretSpin.getVelocity();
            double error = targetVel - currentVel;
            sleep(50);

            tele.addData("error" , error);
            tele.addData("targetVelocity" , targetVel);
            tele.addData("currentVelocity" , currentVel);
            tele.addData("P" , P);
            tele.addData("I" , I);
            tele.addData("D" , D);
            tele.addData("F" , F);
            updateTelemetry(tele);
        }
    }
}
