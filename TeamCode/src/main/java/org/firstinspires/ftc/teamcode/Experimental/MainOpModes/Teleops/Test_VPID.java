package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;




import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.degreesToOuttakeTurretServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOPBlue.turretAngleOverride;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;



@Config
@TeleOp(name ="VPID", group = "Linear OpMode")
public class Test_VPID extends LinearOpMode {
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
    //double ticksPerSecond = turretspin1.getVelocity();
    private static final double v_TICKS_PER_REV = 28.0;
    private static final int v_BUFFER_SIZE = 100;
    private static double[] v_position_history = new double[v_BUFFER_SIZE];
    private static long[] v_time_history = new long[v_BUFFER_SIZE];
    private static int v_history_idx = 0;
    private static double v_measuredRPM = 0.0;
    public double MeasureRPM(DcMotorEx motor) {
        double current_pos = motor.getCurrentPosition();

        // 1. Store the new measurement at the current index
        v_time_history[v_history_idx] = System.currentTimeMillis();
        v_position_history[v_history_idx] = current_pos;

        int oldest_idx = (v_history_idx + 1) % v_BUFFER_SIZE;

        double newest_pos = current_pos;
        long newest_timeMs = System.currentTimeMillis();

        double oldest_pos = v_position_history[oldest_idx];
        long oldest_timeMs = v_time_history[oldest_idx];

        // Making the subtraction for delta time
        long deltaT_ms = newest_timeMs - oldest_timeMs;
        double deltaP_ticks = newest_pos - oldest_pos;
        if (deltaT_ms > 50) {
            v_measuredRPM = (deltaP_ticks / v_TICKS_PER_REV) * (60000.0 / deltaT_ms);
        } else if (deltaT_ms <= 0) {
            v_measuredRPM = 0.0;
        }

        v_history_idx = (v_history_idx + 1) % v_BUFFER_SIZE;

        return v_measuredRPM;
    }


    @Override
    public void runOpMode() {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretspin1= hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        turretspin1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretspin1.setDirection(DcMotorSimple.Direction.REVERSE);
        turretspin2= hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");
        turretspin2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //double rpm = ticksPerSecond * 60 / TICKS_PER_REV;

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

        if(v_pos_timer.milliseconds() >= 100) {
            v_current_pos = turretspin1.getCurrentPosition();
            double delta_pos = v_current_pos - v_last_pos;
            delta_pos /= 28;
            v_last_pos = v_current_pos;
            v_current_rpm = delta_pos * 600;
            v_pos_timer.reset();
        }
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


