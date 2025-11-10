package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Test", group = "Tests")
public class LimelightTest extends OpMode {

    private Limelight3A limelight3A;
    private double old_pos_y_pruple = 0;
    private double ball_counter = 0;

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
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();

        if(!limelight3A.isRunning()) limelight3A.start();

        if (llResult != null) {
            // Basic vision data
            telemetry.addData("X offset", llResult.getTx());
            telemetry.addData("Y offset", llResult.getTy());
            telemetry.addData("Target area", llResult.getTa());

            // Optional: Python pipeline data
            double[] pythonOutputs = llResult.getPythonOutput();
            if (pythonOutputs != null) {
                telemetry.addLine("Python Output:");

                double pos_y = pythonOutputs[1];
                telemetry.addData("y_pos: ", pos_y);
                telemetry.addData("y_old_pos: ", old_pos_y_pruple);
                if(old_pos_y_pruple > 170 && pos_y - old_pos_y_pruple > 50){
                    ball_counter += 1;
                }
                old_pos_y_pruple = pos_y;

                telemetry.addData("ball counter: ", ball_counter);

                for (int i = 0; i < 8; i++) {
                    telemetry.addData("Value " + i, pythonOutputs[i]);
                }
            } else {
                telemetry.addLine("No Python output received.");
            }
        } else {
            telemetry.addLine("No Limelight data yet...");
        }
        telemetry.update();
    }

}
