package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="CarFestGearBox", group="Testing")
public class CarFestGearBox extends LinearOpMode {

    public static double multi = 1;
    public static double override = 0;
    public static boolean lock = false;
    public static double power = 0;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor0");
        MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            double input = gamepad1.left_stick_y;
            power = override != 0 ? override : input;
            power *= multi;
            if (!lock) {
                motor.setPower(power);
                tel.addData("power", power);
            }
            if (gamepad1.crossWasPressed()) lock = !lock;
            tel.update();
        }
    }
}
