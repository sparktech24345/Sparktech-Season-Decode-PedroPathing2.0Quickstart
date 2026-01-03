package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.intakeSorterServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.turretAngleServoName;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

@Config
@TeleOp( name = "TESTnush", group = "Linear OpMode")
public class TESTnush extends LinearOpMode {
    DcMotorEx turretSpinLeft;
    DcMotorEx turretSpinRight;
    Servo intakeSorterServo;
    protected RobotController robot;
    public static double P = 180;
    public static double D = 18;
    public static double I = 0;
    public static double F = 15;
    public static double v_target_Velocity = 0;
    public static double pow = 0;
    public static double servoPos = 0.8;

    @Override
    public void runOpMode() {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretSpinLeft = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        turretSpinLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretSpinLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        turretSpinRight = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");
        turretSpinRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients PIDFCoefficients = new PIDFCoefficients(P,I,D,F);
        turretSpinRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,PIDFCoefficients);


        DcMotor motor = hardwareMap.dcMotor.get("intakeMotor");
        Servo servo2 = hardwareMap.get(Servo.class, turretAngleServoName);
        intakeSorterServo = hardwareMap.get(Servo.class, intakeSorterServoName);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            //robot.executeNow(new StateAction("TurretAngle", "DEFAULT"));
            servo2.setPosition(servoPos);
            motor.setPower(gamepad1.left_stick_y);

            turretSpinRight.setVelocity(v_target_Velocity);
            turretSpinLeft.setPower(turretSpinRight.getPower());
            //turretSpinR.setVelocity(targetVel);
            double currentVelLeft = turretSpinLeft.getVelocity();
            double currentVelRight = turretSpinRight.getVelocity();
            double errorLeft = v_target_Velocity - currentVelLeft;
            double errorRight = v_target_Velocity - currentVelRight;
            turretSpinRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(P,I,D,F));
            intakeSorterServo.setPosition(154.8/360);
            tele.addData("targetVelocity", v_target_Velocity);
            tele.addData("error RIGHT", currentVelRight);
            tele.addData("error LEFT", currentVelLeft);
            tele.addData("motor pow right", turretSpinRight.getPower());
            tele.addData("motor pow left", turretSpinLeft.getPower());
            tele.addData("P",P);
            tele.addData("I",I);
            tele.addData("D",D);
            tele.addData("F",F);
            tele.update();

        }

    }
}
