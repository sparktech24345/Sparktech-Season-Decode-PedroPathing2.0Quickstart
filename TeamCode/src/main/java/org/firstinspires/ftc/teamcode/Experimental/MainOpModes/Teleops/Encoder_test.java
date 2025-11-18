package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;


@TeleOp(name = "Test Encoder", group = "Linear OpMode")
public class Encoder_test extends LinearOpMode {
 private Encoder encoder = null;

    @Override
    public void runOpMode(){
        MultipleTelemetry tele= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        encoder = new Encoder(hardwareMap.get(DcMotor.class, "backpurple"));
        encoder.getMotorInstance().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.getMotorInstance().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
         tele.addData("encoder pos", encoder.getEncoderPosition());
         tele.update();
        }
    }
}
