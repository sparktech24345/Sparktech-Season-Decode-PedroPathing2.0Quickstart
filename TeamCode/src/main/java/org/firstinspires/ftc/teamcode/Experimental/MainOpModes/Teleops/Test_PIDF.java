package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name="PIDF" ,group="LinearOpMode")
public class Test_PIDF extends LinearOpMode {
    DcMotorEx turretSpinL;
    DcMotorEx turretSpinR;
    public static double P = 180;
    public static double D = 18;
    public static double I = 0;
    public static double F = 15;
    public static double currentVelLeft;
    public static double currentVelRight;
    public static double targetVel;

    @Override
    public void runOpMode(){
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretSpinL = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        turretSpinL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretSpinL.setDirection(DcMotorSimple.Direction.REVERSE);
        turretSpinR = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");
        turretSpinR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients PIDFCoefficients = new PIDFCoefficients(P,I,D,F);
        turretSpinL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);
        turretSpinR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            turretSpinL.setVelocity(targetVel);
            turretSpinR.setVelocity(targetVel);
            currentVelLeft = turretSpinL.getVelocity();
            currentVelRight = turretSpinR.getVelocity();
            double errorLeft = targetVel - currentVelLeft;
            double errorRight = targetVel - currentVelRight;
            turretSpinL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(P,I,D,F));
            turretSpinR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(P,I,D,F));
            sleep(150);

            tele.addData("errorLeft" , errorLeft);
            tele.addData("errorRight" , errorRight);
            tele.addData("targetVelocity" , targetVel);
            tele.addData("currentVelocityLeft" , currentVelLeft);
            tele.addData("currentVelocityRight" , currentVelRight);
            tele.addData("left motor power" , turretSpinL.getPower());
            tele.addData("right motor power" , turretSpinR.getPower());
            tele.addData("P" , P);
            tele.addData("I" , I);
            tele.addData("D" , D);
            tele.addData("F" , F);
            tele.update();
        }
    }
}
