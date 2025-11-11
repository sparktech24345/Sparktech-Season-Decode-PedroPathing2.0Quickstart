package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Limelight Test", group = "Tests")
public class LimelightTest extends OpMode {

    private Limelight3A limelight3A;
    private double old_pos_y_pruple = 0;
    private double ball_counter = 0;
    private double old_Ty = 0;
    private double old_Tx = 0;
    private boolean doOnce = true;
    ElapsedTime timp = new ElapsedTime();

    private long timeLime = 0;

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

        timeLime = System.currentTimeMillis();
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

            if(doOnce){
                old_Tx = llResult.getTx();
                old_Ty = llResult.getTy();
                doOnce = false;
            }

            if(old_Tx == llResult.getTx() && llResult.getTy() == old_Ty){
                limelight3A.stop();
                if (timp.milliseconds()<100) {
                    limelight3A.start();
                }
            }

            // Optional: Python pipeline data
            double[] pythonOutputs = llResult.getPythonOutput();
            if (pythonOutputs != null) {
                telemetry.addLine("Python Output:");

                double pos_y = pythonOutputs[1];
                telemetry.addData("y_pos: ", pos_y);
                telemetry.addData("y_old_pos: ", old_pos_y_pruple);
                if(old_pos_y_pruple > 190 && old_pos_y_pruple - pos_y > 50 && System.currentTimeMillis() - timeLime > 600){
                    ball_counter += 1;
                    timeLime = System.currentTimeMillis();
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
