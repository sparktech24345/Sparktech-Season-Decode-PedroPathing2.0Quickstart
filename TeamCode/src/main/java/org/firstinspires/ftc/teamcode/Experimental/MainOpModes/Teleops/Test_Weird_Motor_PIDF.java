package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.StolenMotorClass.Motor;

@Config
@Disabled
@TeleOp(name="Test_Weird_Motor_PIDF" ,group="LinearOpMode")
public class Test_Weird_Motor_PIDF extends LinearOpMode {
    Motor fancyMotorL;
    Motor fancyMotorR;
    public static double P = 8;
    public static double D = 0.03;
    public static double I = 0;
    public static double ks = 0;
    public static double kv = 1.2;
    public static double ka = 0.08;
    public static double currentVelLeft;
    public static double currentVelRight;
    public static double targetVel;
    public static double magicDivideNumber = 2800;
    public static double buffer = 1;

    @Override
    public void runOpMode(){
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fancyMotorL = new Motor(hardwareMap,"turretFlyWheelMotorLeft", Motor.GoBILDA.BARE);
        fancyMotorR = new Motor(hardwareMap,"turretFlyWheelMotorRight", Motor.GoBILDA.BARE);

        fancyMotorL.setRunMode(Motor.RunMode.VelocityControl);
        fancyMotorR.setRunMode(Motor.RunMode.RawPower);
        fancyMotorR.setInverted(true);


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            fancyMotorL.setBuffer(buffer);
            fancyMotorR.setBuffer(buffer);

            fancyMotorL.setVeloCoefficients(P,I,D);
            fancyMotorL.setFeedforwardCoefficients(ks,kv,ka);
            fancyMotorL.set(targetVel / magicDivideNumber);
            double power = fancyMotorL.get();
            fancyMotorR.set(power);

            currentVelLeft = fancyMotorL.getCorrectedVelocity();
            currentVelRight = fancyMotorR.getCorrectedVelocity();

            double errorLeft = targetVel - currentVelLeft;
            double errorRight = targetVel - currentVelRight;

            sleep(50);

            tele.addData("errorLeft" , errorLeft);
            tele.addData("errorRight" , errorRight);
            tele.addData("targetVelocity" , targetVel);
            tele.addData("currentVelocityLeft" , currentVelLeft);
            tele.addData("currentVelocityRight" , currentVelRight);
            tele.addData("left motor power" , fancyMotorL.get());
            tele.addData("right motor power" , fancyMotorR.get());
            tele.addData("P" , P);
            tele.addData("I" , I);
            tele.addData("D" , D);
            tele.update();
        }
    }
}
