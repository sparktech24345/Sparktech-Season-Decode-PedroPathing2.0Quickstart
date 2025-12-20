package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AbsoluteAnalogEncoder;

@Disabled
@Config
@TeleOp(name = "servoSpin", group = "Linear OpMode")
public class servoSpin extends LinearOpMode {
    public static double servoPos = 30/360;
    public static double pow = 0;
    AbsoluteAnalogEncoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo servo1 = hardwareMap.get(CRServo.class, "turretrotateleft");
        CRServo servo2 = hardwareMap.get(CRServo.class, "turretrotateright");

        encoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "leftturretreader"));

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            servo1.setPower(gamepad1.left_stick_x + pow);
            servo2.setPower(gamepad1.left_stick_x + pow);
            //servo2.setPower(gamepad1.right_stick_x);

            tel.addData("servo 1 powe",servo1.getPower());
            tel.addData("servo 2 powe",servo2.getPower());
            tel.addData("encodeh ",Math.toDegrees(encoder.getCurrentPosition()));

            tel.update();
        }
    }
}
