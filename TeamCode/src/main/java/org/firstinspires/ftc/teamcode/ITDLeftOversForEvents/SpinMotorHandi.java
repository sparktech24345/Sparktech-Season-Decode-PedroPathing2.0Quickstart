package org.firstinspires.ftc.teamcode.ITDLeftOversForEvents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "SPIININ handi", group = "Linear OpMode")
public class SpinMotorHandi extends LinearOpMode {
    double lastPos =0;
    double pos =0;
    long timer=0;
    double diff=0;

    @Override
    public void runOpMode() throws InterruptedException {


        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor motor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor motor1 = hardwareMap.dcMotor.get("turretFlyWheelMotorLeft");
        DcMotor motor2 = hardwareMap.dcMotor.get("turretFlyWheelMotorRight");
        timer = System.currentTimeMillis();

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
            motor1.setPower(gamepad1.right_stick_y);
            motor2.setPower(-gamepad1.right_stick_y);

            if(timer + 1000 < System.currentTimeMillis()) {
                pos = motor.getCurrentPosition();
                diff = pos - lastPos;
                lastPos = pos;
                timer = System.currentTimeMillis();
                diff /= 28;
            }

            tel.addData("Rotation per sec",diff);
            tel.addData("Rotation per min",diff*60);
            tel.update();

        }
    }
}
