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
    double current_ms = 0;

    private double kp = 0;
    private double kd = 0;
    private double last_error = 0;

    private double inc_res = 1;
    private int idx = 0;

    private DcMotor target_motor;

    private double calculate_power() {
        double error = target_rpm - current_rpm;
        double p = kp * error;
        double d =  kd * (error - last_error);
        last_error = error;
        return clamp(p + d, -1, 1);
    }

    @Override
    public void runOpMode() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {}
        };
        robot.init(OpModes.TeleOP);
        robot.UseDefaultMovement();

        target_motor = hardwareMap.get(DcMotor.class, GlobalStorage.intakeSpinName);
        target_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()) {
            robot.init_loop();
        }
        pos_timer.reset();
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) switch (idx) {
                case 0: target_rpm += inc_res; break;
                case 1: kp += inc_res; break;
                case 2: kd += inc_res; break;
            }
            if (gamepad1.aWasPressed()) switch (idx) {
                case 0: target_rpm -= inc_res; break;
                case 1: kp -= inc_res; break;
                case 2: kd -= inc_res; break;
            }
            if (gamepad1.xWasPressed()) inc_res *= 10;
            if (gamepad1.bWasPressed()) inc_res /= 10;
            if (gamepad1.dpadUpWasPressed()) idx = (idx + 1) % 3;
            if (gamepad1.dpadDownWasPressed()) if (idx > 0) --idx;

            current_pos = target_motor.getCurrentPosition();
            current_ms = pos_timer.milliseconds();
            current_rpm = (current_pos - last_pos) / current_ms;

            target_motor.setPower(calculate_power());

            Telemetry tele = RobotController.telemetryInstance;
            tele.addData("Current", () -> {
                switch(idx) {
                    case 0: return "target_rpm";
                    case 1: return "kp";
                    case 2: return "kd";
                }
                return "none";
            });
            tele.addData("target_rpm", target_rpm);
            tele.addData("increment_res", inc_res);
            tele.addData("kp", kp);
            tele.addData("kd", kd);
            tele.addData("rpm", current_rpm);
            tele.addData("pos", current_pos);
            last_pos = current_pos;

            robot.loop();
            pos_timer.reset();
        }
    }
}
