package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Visual.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagWecam extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    long lastTime=0;
    MultipleTelemetry tel;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

        tel.addData("diff", (double)System.nanoTime() - lastTime);
        lastTime = System.nanoTime();
        tel.update();
        //telemetry.addData("id20 String", id20.toString());
    }
}
