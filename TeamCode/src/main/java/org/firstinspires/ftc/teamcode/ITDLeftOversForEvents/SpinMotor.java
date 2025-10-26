package org.firstinspires.ftc.teamcode.ITDLeftOversForEvents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "SPIININ", group = "Linear OpMode")
public class SpinMotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor motor = hardwareMap.dcMotor.get("themotor");


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
