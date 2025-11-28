package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;

@TeleOp(name = "Limelight Test", group = "Tests")
public class LimelightTest extends OpMode {


    private Limelight3A limelight3A;
    private double old_pos_y_purple = 0;
    private double ball_counter = 0;
    private double old_Ty = 0;
    private double old_Tx = 0;
    private boolean doOnce = true;
    ElapsedTime timp = new ElapsedTime();

    private long timeLime = 0;


    /// ----------------- Color Sensor Stuff ------------------
    private NormalizedColorSensor colorSensorGreen;
    private NormalizedColorSensor colorSensorPurple;
    private NormalizedColorSensor colorSensorLaunch;
    private NormalizedRGBA greenSensorColors;
    private NormalizedRGBA purpleSensorColors;
    private NormalizedRGBA launchSensorColors;
    private DcMotorEx intakeMotor;


    final float[] hsvValuesGreen = new float[3];
    final float[] hsvValuesPurple = new float[3];

    final float[] hsvValuesLaunch = new float[3];
    /// --------------------------------------------------------

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(1);
        limelight3A.pipelineSwitch(0);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100); // poll 100 times per second
        limelight3A.start();

        telemetry.addLine("Limelight initialized and streaming...");
        telemetry.update();

        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple = hardwareMap.get(NormalizedColorSensor.class, "purplesensor");
        colorSensorLaunch = hardwareMap.get(NormalizedColorSensor.class, "launchsensor");

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakemotor");


        timeLime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        HandleColors();
        LLResult llResult = limelight3A.getLatestResult();

        intakeMotor.setPower(gamepad1.left_stick_y);

//        if(!limelight3A.isRunning()) limelight3A.start();
//        if (llResult != null) {
//            // Basic vision data
//            telemetry.addData("X offset", llResult.getTx());
//            telemetry.addData("Y offset", llResult.getTy());
//            telemetry.addData("Target area", llResult.getTa());
//
//            if(doOnce){
//                old_Tx = llResult.getTx();
//                old_Ty = llResult.getTy();
//                doOnce = false;
//            }
//
//            if(old_Tx == llResult.getTx() && llResult.getTy() == old_Ty){
//                limelight3A.stop();
//                if (timp.milliseconds()>=100) {
//                    limelight3A.start();
//                }
//            }
//
//            // Optional: Python pipeline data
//            double[] pythonOutputs = llResult.getPythonOutput();
//            if (pythonOutputs != null) {
//                telemetry.addLine("Python Output:");
//
//                double pos_y = pythonOutputs[1];
//                telemetry.addData("y_pos: ", pos_y);
//                telemetry.addData("y_old_pos: ", old_pos_y_purple);
//                if(old_pos_y_purple > 190 && old_pos_y_purple - pos_y > 50 && System.currentTimeMillis() - timeLime > 600){
//                    ball_counter += 1;
//                    timeLime = System.currentTimeMillis();
//                }
//                old_pos_y_purple = pos_y;
//
//                telemetry.addData("ball counter: ", ball_counter);
//
//                for (int i = 0; i < 8; i++) {
//                    telemetry.addData("Value " + i, pythonOutputs[i]);
//                }
//            } else {
//                telemetry.addLine("No Python output received.");
//            }
//        } else {
//            telemetry.addLine("No Limelight data yet...");
//        }
        double tx = llResult.getTx();

        telemetry.addData("tx", tx);


        telemetry.update();
    }

    private void HandleColors() {


        greenSensorColors =colorSensorGreen.getNormalizedColors();
        purpleSensorColors =colorSensorPurple.getNormalizedColors();
        launchSensorColors = colorSensorLaunch.getNormalizedColors();

        Color.colorToHSV(greenSensorColors.toColor(), hsvValuesGreen);
        Color.colorToHSV(purpleSensorColors.toColor(), hsvValuesPurple);
        Color.colorToHSV(launchSensorColors.toColor(), hsvValuesLaunch);


        greenSensorBall = BallColorSet_Decode.getColorForTurret(greenSensorColors);
        purpleSensorBall1 = BallColorSet_Decode.getColorForTurret(purpleSensorColors);
        launchSensorBall = BallColorSet_Decode.getColorForTurret(launchSensorColors);


        telemetry.addData("G_RED",(double)greenSensorColors.red * 10000.0);
        telemetry.addData("G_BLUE",(double)greenSensorColors.blue * 10000.0);
        telemetry.addData("G_GREEN",(double)greenSensorColors.green * 10000.0);

        telemetry.addData("P_RED",(double)purpleSensorColors.red * 10000.0);
        telemetry.addData("P_BLUE",(double)purpleSensorColors.blue * 10000.0);
        telemetry.addData("P_GREEN",(double)purpleSensorColors.green * 10000.0);

        telemetry.addData("LAUNCH_RED",(double)launchSensorColors.red * 10000.0);
        telemetry.addData("LAUNCH_BLUE",(double)launchSensorColors.blue * 10000.0);
        telemetry.addData("LAUNCH_GREEN",(double)launchSensorColors.green * 10000.0);


        greenSensorBall = BallColorSet_Decode.NoBall;
        purpleSensorBall1 = BallColorSet_Decode.NoBall;
        launchSensorBall = BallColorSet_Decode.NoBall;

        telemetry.addData("GREEN_SENSOR_BALL", greenSensorBall);
        telemetry.addData("PURPLE_SENSOR_BALL", purpleSensorBall1);
        telemetry.addData("LAUNCH_SENSOR_BALL", launchSensorBall);
    }

}
