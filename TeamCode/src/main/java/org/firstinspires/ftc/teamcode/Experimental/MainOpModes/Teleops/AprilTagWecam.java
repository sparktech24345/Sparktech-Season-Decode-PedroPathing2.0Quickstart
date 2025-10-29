package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Visual.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagWecam extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
//        aprilTagWebcam.displayDetectionTelemetry(id20);
        telemetry.addData("id20 String", id20.toString());
    }
}
