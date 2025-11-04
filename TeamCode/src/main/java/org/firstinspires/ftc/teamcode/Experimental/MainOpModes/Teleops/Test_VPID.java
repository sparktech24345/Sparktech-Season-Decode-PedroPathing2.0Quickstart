package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;




import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name ="VPID", group = "Linear OpMode")
public class Test_VPID extends LinearOpMode {
    DcMotorEx turretspin1;
    public static double ks = 0.012;//TODO:de schimbat valori
    public static double kV = 0.00009;
    public static double kp = 0.00006;
    public static double error = 0;
    public static double current_rpm = 0;
    ElapsedTime pos_timer= new ElapsedTime();
    public static double current_pos = 0;
    public static double last_pos = 0;
    public static double target_rpm = 2000;
    //double ticksPerSecond = turretspin1.getVelocity();
    private static final double TICKS_PER_REV = 28.0;


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
        error = target_rpm - current_rpm;
        double power = ks + kV * turretspin1.getVelocity() + kp * error;
        turretspin1.setPower(Math.max(-1,Math.min(power,1)));

            tele.addData("targetrpm", target_rpm);
            tele.addData("currentrpm", current_rpm);
            tele.addData("power", power);
            tele.addData("measured velocity", turretspin1.getVelocity());
            tele.addData("error", error);
            tele.addData("ks",ks);
            tele.addData("kV",kV);
            tele.addData("kp",kp);
            tele.update();

        }
    }
}


