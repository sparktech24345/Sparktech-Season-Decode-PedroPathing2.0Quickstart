package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.turretAngleServoName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.ActionSequence;
@Config
@TeleOp( name = "TESTnush", group = "Linear OpMode")
public class TESTnush extends LinearOpMode {
    DcMotorEx turretspin1;
    DcMotorEx turretspin2;
    protected RobotController robot;
    public static double v_ks = 0;//TODO:de schimbat valori
    public static double v_kV = 0.00001;
    public static double v_kp = 0.0015;
    public static double multiplier = 0.2;
    public static double v_errorR = 0;
    public static double v_errorL = 0;
    public static double v_current_rpm = 0;
    ElapsedTime v_pos_timer= new ElapsedTime();
    public static double v_current_pos = 0;
    public static double v_last_pos = 0;
    public static double v_target_Velocity = 1030;

    @Override
    public void runOpMode() {

        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretspin1= hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        turretspin1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretspin1.setDirection(DcMotorSimple.Direction.REVERSE);
        turretspin2= hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");
        turretspin2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor motor = hardwareMap.dcMotor.get("intakeMotor");
        Servo servo2 = hardwareMap.get(Servo.class, turretAngleServoName);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            //robot.executeNow(new StateAction("TurretAngle", "DEFAULT"));
            servo2.setPosition(0.8);
            if(v_pos_timer.milliseconds() >= 100) {
                v_current_pos = turretspin1.getCurrentPosition();
                double delta_pos = v_current_pos - v_last_pos;
                delta_pos /= 28;
                v_last_pos = v_current_pos;
                v_current_rpm = delta_pos * 600;
                v_pos_timer.reset();
            }
            motor.setPower(gamepad1.left_stick_y);

            double velocityL = turretspin1.getVelocity();
            v_errorL = v_target_Velocity - velocityL;
            double v_powerL = v_ks + v_kV * velocityL + v_kp * v_errorL;
            turretspin1.setPower(Math.max(-1,Math.min(v_powerL,1)));

            double velocityR = turretspin2.getVelocity();
            v_errorR = v_target_Velocity - velocityR;
            double v_powerR = v_ks + v_kV * velocityR + v_kp * v_errorR;
            turretspin2.setPower(Math.max(-1,Math.min(v_powerR,1)));

            tele.addData("targetVelocity", v_target_Velocity);
            tele.addData("power LEFT", v_powerL);
            tele.addData("power RIGHT", v_powerR);
            tele.addData("measured velocity LEFT", velocityL);
            tele.addData("measured velocity RIGHT", velocityR);
            tele.addData("error RIGHT", v_errorR);
            tele.addData("error LEFT", v_errorL);
            tele.addData("ks",v_ks);
            tele.addData("kV",v_kV);
            tele.addData("kp",v_kp);
            tele.update();

        }

    }
}
