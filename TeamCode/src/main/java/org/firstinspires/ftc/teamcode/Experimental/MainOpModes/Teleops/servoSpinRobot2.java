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

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@TeleOp(name = "servoSpinRobotv2", group = "Linear OpMode")
public class servoSpinRobot2 extends LinearOpMode {
    public static double P = 185;
    public static double I = 0.02;
    public static double D = 17.8;
    public static double F = 15;
    public static double currentVel1;
    public static double currentVel2;
    public static double targetVel;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "turretspinleft");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "turretspinright");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients coeff = new PIDFCoefficients(P, I, D, F);

        PIDFCoefficients PIDFCoefficients = new PIDFCoefficients(P,I,D,F);
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            coeff = new PIDFCoefficients(P, I, D, F);

            motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);
            motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);

            motor1.setVelocity(targetVel);
            motor2.setVelocity(targetVel);

            currentVel1 = motor1.getVelocity();
            currentVel2 = motor2.getVelocity();

            double error1 = targetVel - currentVel1;
            double error2 = targetVel - currentVel2;

            tel.addData("servo 1 powe",motor1.getPower());
            tel.addData("servo 2 powe",motor2.getPower());
            tel.addData("error1" , error1);
            tel.addData("error2" , error2);
            tel.addData("avg error" , (error1 + error2) / 2);
            tel.addData("targetVelocity" , targetVel);
            tel.addData("currentVelocity1" , currentVel1);
            tel.addData("currentVelocity2" , currentVel2);
            tel.addData("P" , P);
            tel.addData("I" , I);
            tel.addData("D" , D);
            tel.addData("F" , F);
            //tel.addData("encodeh ",Math.toDegrees(encoder.getCurrentPosition()));

            tel.update();
        }
    }
}
