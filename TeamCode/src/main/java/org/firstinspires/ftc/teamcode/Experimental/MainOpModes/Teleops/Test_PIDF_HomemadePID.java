package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.intakeSorterServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.turretAngleServoName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.StolenMotorClass.SimpleMotorFeedforward;

@Config
@Disabled
@TeleOp(name="Test_PIDF_HomemadePID" ,group="Linear OpMode")
public class Test_PIDF_HomemadePID extends LinearOpMode {
    SimpleMotorFeedforward feedforward;
    DcMotorEx turretSpinL;
    DcMotorEx turretSpinR;
    PIDcontroller piDcontroller;
    public static double P = 0.0055;
    public static double D = 0;
    public static double I = 0;
    public static double F = 0.0004;
    public static double currentVelLeft;
    public static double currentVelRight;
    public static double targetVel;
    public static double turretAngle = 275;
    public static double motorPow = 0;
    public static double powerOverride = 0;

    @Override
    public void runOpMode() {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretSpinL = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        turretSpinL.setDirection(DcMotorSimple.Direction.REVERSE);
        piDcontroller = new PIDcontroller(P, I, D);


        Servo servo2 = hardwareMap.get(Servo.class, turretAngleServoName);
        servo2.setPosition(turretAngle / 360);
        DcMotorSimple intakeMotor = hardwareMap.get(DcMotorSimple.class, intakeMotorName);
        Servo servo3 = hardwareMap.get(Servo.class, intakeSorterServoName);
        servo3.setPosition(0.17);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            intakeMotor.setPower(motorPow);
            servo2.setPosition(turretAngle/360);

            currentVelLeft = turretSpinL.getVelocity();

            double errorLeft = targetVel - currentVelLeft;


            double power = piDcontroller.calculate(targetVel,currentVelLeft) + targetVel * F;
            piDcontroller.setConstants(P, I, D);

            if(powerOverride != 0) power = powerOverride;

            turretSpinL.setPower(power);

            sleep(50);

            tele.addData("errorLeft" , errorLeft);
            tele.addData("targetVelocity" , targetVel);
            tele.addData("currentVelocityLeft" , currentVelLeft);
            tele.addData("currentVelocityRight" , currentVelRight);
            tele.addData("left motor power" , turretSpinL.getPower());
            tele.addData("left motor velocity smth" , turretSpinR.getVelocity(AngleUnit.DEGREES));
            tele.addData("amperaj R" , turretSpinR.getCurrent(CurrentUnit.AMPS));
            tele.addData("amperaj L" , turretSpinL.getCurrent(CurrentUnit.AMPS));
            tele.addData("P" , P);
            tele.addData("I" , I);
            tele.addData("D" , D);
            tele.addData("F" , F);
            tele.update();
        }
    }
}
