package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

@TeleOp(name = "TEST VOLTAJ", group = "Tests")
public class test_voltaj_motor_PE_ITD extends LinearOpMode {

    private RobotController robot;
    private double current_rpm = 0;
    private double target_rpm = 0;

    private double current_pos = 0;
    private double last_pos = 0;
    ElapsedTime pos_timer = new ElapsedTime();
    double current_s = 0;

    private double kp = 0;
    private double kd = 0;
    private double last_error = 0;

    private double inc_res = 1;
    private int idx = 0;

    private DcMotor target_motor;

    private DcMotor encoder;

    private boolean override= false;

    private double maxrpm = 6000;

    private double calculate_power() {
        return target_rpm / maxrpm;
//        double error = target_rpm - current_rpm;
//        double p = kp * error;
//        double d =  kd * (error - last_error);
//        last_error = error;
//        return clamp(p + d, -1, 1);
    }

    @Override
    public void runOpMode() {
        MultipleTelemetry tele= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotController(hardwareMap, tele , gamepad1, gamepad2) {
            @Override
            public void main_loop() {}
        };
        robot.init(OpModes.TeleOP);
        robot.UseDefaultMovement();

        target_motor = hardwareMap.get(DcMotor.class, "intakespin");
        target_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        encoder=  hardwareMap.get(DcMotor.class, "outakerightmotor");
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeInInit()) {
            robot.init_loop();
        }
        pos_timer.reset();
        while (opModeIsActive()) {
            robot.loop();
            if (gamepad1.yWasPressed()) target_rpm += inc_res;
            if (gamepad1.aWasPressed()) target_rpm -= inc_res;
            if (gamepad1.xWasPressed()) inc_res *= 10;
            if (gamepad1.bWasPressed()) inc_res /= 10;
            if (gamepad1.dpadLeftWasPressed()) override = !override;
            if (gamepad1.dpadUpWasPressed()) maxrpm += inc_res;
            if (gamepad1.dpadDownWasPressed()) maxrpm -= inc_res;

            double power = calculate_power();
            if (override) power = 1;
            target_motor.setPower(power);

            // switch(idx) {
            //     case 0: tele.addData("current", "target_rpm"); break;
            //     case 1: tele.addData("current", "kp"); break;
            //     case 2: tele.addData("current", "kd");break;
            // }

            // tele.addData("power", power);
            tele.addData("target_rpm", target_rpm);
            tele.addData("increment_res", inc_res);
            // tele.addData("kp", kp);
            // tele.addData("kd", kd);
            tele.addData("rpm", current_rpm);
            // tele.addData("pos", current_pos);
            tele.addData("ms", current_s);
            tele.update();
            if (pos_timer.milliseconds() >= 100) {
                current_pos = encoder.getCurrentPosition();
                double diff = current_pos - last_pos;
                diff /= 28;
                current_s = pos_timer.seconds();
                current_rpm = diff * 600;
                last_pos = current_pos;
                pos_timer.reset();
            }
        }
    }
}
