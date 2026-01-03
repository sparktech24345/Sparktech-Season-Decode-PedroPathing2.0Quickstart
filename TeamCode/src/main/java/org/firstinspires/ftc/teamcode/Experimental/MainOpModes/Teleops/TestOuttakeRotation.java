package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.turretRotationMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

@Config
@TeleOp(name="TestOuttakeRotation" ,group="LinearOpMode")
public class TestOuttakeRotation extends LinearOpMode {
    DcMotorEx turretRotate;
    public static double P = 0;
    public static double D = 0;
    public static double I = 0;
    public static double F = 0;
    public static double currentVel;
    public static double currentVelRight;
    public static int targetPos;
    public static double motorPowMultiplier = 1.1;
    public static boolean runToPos = false;
    private PIDcontroller piDcontroller = new PIDcontroller();

    @Override
    public void runOpMode(){
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretRotate = hardwareMap.get(DcMotorEx.class, turretRotationMotorName);
        turretRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            piDcontroller.setConstants(P,I,D);
            currentVel = turretRotate.getVelocity();
            double motorPow = piDcontroller.calculate(targetPos,turretRotate.getCurrentPosition()/2.62);
            if(Math.abs(currentVel) < 0.00001) motorPow *= motorPowMultiplier;
            turretRotate.setPower(motorPow);

            sleep(50);

            tele.addData("error" , targetPos - turretRotate.getCurrentPosition());
            tele.addData("current position" , turretRotate.getCurrentPosition() / 2.62);
            tele.addData("currentVelocity" , currentVel);
            tele.addData("motor power" , turretRotate.getPower());
            tele.addData("P" , P);
            tele.addData("I" , I);
            tele.addData("D" , D);
            tele.addData("F" , F);
            tele.update();
        }
    }
}
