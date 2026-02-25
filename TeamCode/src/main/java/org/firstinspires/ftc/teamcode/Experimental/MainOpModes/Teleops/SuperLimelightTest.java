package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.backLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.backRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.frontLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.frontRightName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Super Limelight", group = "Tests")
public class SuperLimelightTest extends OpMode {
    List<Integer> ballQueue;
    Limelight3A limelight;
    private DcMotor RFDrive;
    private DcMotor RBDrive;
    private DcMotor LFDrive;
    private DcMotor LBDrive;
    MultipleTelemetry tel;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        RFDrive = hardwareMap.get(DcMotor.class, frontRightName);
        LFDrive = hardwareMap.get(DcMotor.class, frontLeftName);
        RBDrive = hardwareMap.get(DcMotor.class, backRightName);
        LBDrive = hardwareMap.get(DcMotor.class, backLeftName);

        RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.setPollRateHz(100);   // retrieves info 100 times per second
        limelight.start();
        limelight.pipelineSwitch(8);
        limelight.reloadPipeline();
    }


    @Override
    public void loop() {
        double vertical     = -gamepad1.left_stick_y;
        double horizontal   = -gamepad1.left_stick_x;
        double pivot        = gamepad1.right_stick_x;

        double FrontRightPow = vertical + horizontal - pivot;
        double BackRightPow  = vertical - horizontal - pivot;
        double FrontLeftPow  = vertical - horizontal + pivot;
        double BackLeftPow   = vertical + horizontal + pivot;

        RFDrive.setPower(FrontRightPow);
        LFDrive.setPower(FrontLeftPow);
        RBDrive.setPower(BackRightPow);
        LBDrive.setPower(BackLeftPow);

        ballQueue = getBallQueue();
        tel.addData("Queue: ", ballQueue);
    }


    private List<Integer> getBallQueue() {
        List<Integer> ballQueue = new ArrayList<>(List.of(0, 0, 0, 0, 0, 0, 0, 0));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            int index = 0;
            for (LLResultTypes.DetectorResult detection : detections) {
                switch (detection.getClassName()) {
                    case "purple": {
                        ballQueue.set(index, 1);
                        break;
                    }
                    case "green": {
                        ballQueue.set(index, 2);
                        break;
                    }
                }
                index++;
            }
        }
        return ballQueue;
    }
}
