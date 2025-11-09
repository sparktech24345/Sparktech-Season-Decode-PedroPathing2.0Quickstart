package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Visual.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;



@Autonomous
public class AprilTagWecam extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    long lastTime=0;
    ElapsedTime detectTime = new ElapsedTime();
    public static double deltaError = 0;
    public static double lastError = 0;
    public static double currentError = 0;
    public static double lastDetectiontX = 0;
    public static double lastDetectiontXError = 0;
    public static Timer lastDetectedTimer = new Timer();
    MultipleTelemetry tel;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        aprilTagWebcam.init(hardwareMap, tel,"Webcam 1");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

        if(id20 != null) {
            double lastDetect = id20.ftcPose.x;
            if(detectTime.milliseconds()<=100){
            double currentDetect = id20.ftcPose.x;
            lastError = currentDetect - lastDetect;
            lastDetect = currentDetect;
            detectTime.reset();
            }
            if(lastError !=0){
                ElapsedTime time0 = new ElapsedTime();
                if(time0.milliseconds()<=100){
                    double currentDetect = id20.ftcPose.x;
                    currentError = currentDetect - lastDetect;
                    deltaError = currentError - lastError;
                    if(deltaError <= 0.01){

                    }
                }
                time0.reset();
            }
        }


//        if(detectTime.milliseconds()<= 100){
//            double currentDetect = id20.ftcPose.x;
//            detectTime.reset();
//            if (lastDetect == currentDetect){
//
//            }
//        }

        tel.addData("diff", (double)System.currentTimeMillis() - lastTime);
        lastTime = System.currentTimeMillis();
        tel.update();
        //telemetry.addData("id20 String", id20.toString());
    }

    public boolean isCameraDetectingTag(AprilTagDetection id20){
        if(id20 != null){
            double pos = id20.ftcPose.x;
            double error = pos - lastDetectiontX;
            boolean toBeReturned = false;
            lastDetectiontX = pos;

            if(error == 0.00){
                toBeReturned = false;
            }
            else toBeReturned =  (lastDetectiontXError != 0);
            lastDetectiontXError = error;
            return toBeReturned;
        }
        else return false;
    }

    public double cameraCorrection(double targetPos , AprilTagDetection id20){
        double correctedTargetPos = 0;
        double ku = Math.sin(targetPos);
        if(isCameraDetectingTag(id20)){
            double pos = id20.ftcPose.x;
            double error = pos - lastDetectiontX;
            lastDetectiontX = pos;
            //de calculat ku
            correctedTargetPos = targetPos + ku * error;
        }
        return correctedTargetPos;
    }
}
