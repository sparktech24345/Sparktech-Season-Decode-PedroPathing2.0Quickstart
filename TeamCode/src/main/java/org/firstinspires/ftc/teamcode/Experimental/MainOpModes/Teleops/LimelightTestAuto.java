package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.intakeMotorName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftSensorColorMultiplier;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;

import java.util.List;

@TeleOp(name = "Limelight Test Auto", group = "Tests")
public class LimelightTestAuto extends OpMode {


    private Limelight3A limelight3A;
    public double camId = 0;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(1);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100);
        limelight3A.start();

    }

    @Override
    public void loop() {
        useCamera();

        telemetry.update();
    }

    public void useCamera() {
        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null) {
            double[] pythonData = llResult.getPythonOutput();
            if (pythonData.length > 0) {
                double firstValue = pythonData[0];

                telemetry.addData("Python Val 1", firstValue);
            }
        }

        telemetry.update();
    }
}