package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;




import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name ="VPID", group = "Linear OpMode")
public class Test_VPID extends LinearOpMode {
    DcMotorEx turretspin1;
    public static double ks = 0;//TODO:de schimbat valori
    public static double kV = 0.00055;
    public static double kp = 0.005;
    public static double multiplier = 0.5;
    public static double error = 0;
    public static double current_rpm = 0;
    ElapsedTime pos_timer= new ElapsedTime();
    public static double current_pos = 0;
    public static double last_pos = 0;
    public static double target_Velocity = 2000;
    //double ticksPerSecond = turretspin1.getVelocity();
    private static final double TICKS_PER_REV = 28.0;
    private static final int BUFFER_SIZE = 100;
    private static double[] position_history = new double[BUFFER_SIZE];
    private static long[] time_history = new long[BUFFER_SIZE];
    private static int history_idx = 0;
    private static double measuredRPM = 0.0;
    public double MeasureRPM(DcMotorEx motor) {
        double current_pos = motor.getCurrentPosition();

        // 1. Store the new measurement at the current index
        time_history[history_idx] = System.currentTimeMillis();
        position_history[history_idx] = current_pos;

        int oldest_idx = (history_idx + 1) % BUFFER_SIZE;

        double newest_pos = current_pos;
        long newest_timeMs = System.currentTimeMillis();

        double oldest_pos = position_history[oldest_idx];
        long oldest_timeMs = time_history[oldest_idx];

        // Making the subtraction for delta time
        long deltaT_ms = newest_timeMs - oldest_timeMs;
        double deltaP_ticks = newest_pos - oldest_pos;
        if (deltaT_ms > 50) {
            measuredRPM = (deltaP_ticks / TICKS_PER_REV) * (60000.0 / deltaT_ms);
        } else if (deltaT_ms <= 0) {
            measuredRPM = 0.0;
        }

        history_idx = (history_idx + 1) % BUFFER_SIZE;

        return measuredRPM;
    }


    @Override
    public void runOpMode(){
        MultipleTelemetry tele= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretspin1= hardwareMap.get(DcMotorEx.class, "turretspin");
        turretspin1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //double rpm = ticksPerSecond * 60 / TICKS_PER_REV;

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){

        if(pos_timer.milliseconds()>=100){
            current_pos = turretspin1.getCurrentPosition();
            double delta_pos = current_pos - last_pos;
            delta_pos/=28;
            last_pos = current_pos;
            current_rpm = delta_pos*600;
            pos_timer.reset();
        }
        double velocity = turretspin1.getVelocity();
        error = target_Velocity - velocity;
        double power = ks + kV * velocity + kp * error;
        turretspin1.setPower(Math.max(-1,Math.min(power,1)));

            tele.addData("targetVelocity", target_Velocity);
            tele.addData("power", power);
            tele.addData("measured velocity", velocity);
            tele.addData("error", error);
            tele.addData("ks",ks);
            tele.addData("kV",kV);
            tele.addData("kp",kp);
            tele.update();

        }
    }
}


