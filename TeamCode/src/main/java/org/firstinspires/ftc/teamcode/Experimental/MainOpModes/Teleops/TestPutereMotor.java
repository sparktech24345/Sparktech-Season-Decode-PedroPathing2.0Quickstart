package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
@Disabled
@TeleOp(name = "TestRpm", group = "Linear OpMode")
public class TestPutereMotor extends LinearOpMode {
    private RobotController robot;
    private double current_pos = 0;
    private double last_pos = 0;
    ElapsedTime pos_timer = new ElapsedTime();
    double current_s = 0;

    double current_rpm = 0;

    private double kp = 0.006;
    private double kd = 0;


    private DcMotor target_motor;

    private double targetPower = 0.2;
    private double currentPower = 0;
    private double error = 0;
    private double lastError = 0;
    private double target_rpm = 2900;
    private double maxrpm = 3500;

    public boolean xIsPressed = false;
    public  boolean yIsPressed = false;



    private double calculate_power() {
        return (target_rpm + error) / maxrpm;
//        double error = target_rpm - current_rpm;
//        double p = kp * error;
//        double d =  kd * (error - last_error);
//        last_error = error;
//        return clamp(p + d, -1, 1);
    }

    @Override
        public void runOpMode(){
        MultipleTelemetry tele= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotController(hardwareMap, tele , gamepad1, gamepad2) {
            @Override
            public void main_loop() {
            }
        };
        robot.init(OpModes.TeleOP);
        robot.UseDefaultMovement();

        target_motor = hardwareMap.get(DcMotor.class, "turretspin");
        target_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()) {
            robot.init_loop();
        }
        pos_timer.reset();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            robot.loop();
            if (gamepad1.x){
                xIsPressed = !xIsPressed;
            }
            if(!gamepad1.x && xIsPressed) {
                xIsPressed = !xIsPressed;
                target_rpm+=100;
            }
            if (gamepad1.y){
                yIsPressed = !yIsPressed;
            }
            if(!gamepad1.y && yIsPressed) {
                yIsPressed = !yIsPressed;
                target_rpm-=100;
            }
            if (gamepad1.aWasPressed()) kp += 0.01;
            if (gamepad1.bWasPressed()) kd += 0.01;

            double power = calculate_power();
            target_motor.setPower(power);

            if (pos_timer.milliseconds() >= 100) {
                current_pos = target_motor.getCurrentPosition();
                double diff = current_pos - last_pos;
                diff /= 28;
                current_s = pos_timer.seconds();
                current_rpm = diff * 600;
                last_pos = current_pos;
                pos_timer.reset();
            }
            error = target_rpm - current_rpm;
            tele.addData("power", power*1000);
            tele.addData("rpm", target_rpm);
            tele.addData("currentRPM" , current_rpm);
            tele.addData("error" ,error);
            tele.update();
        }
    }
}


