package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Limelight Test", group = "Tests")
public class LimelightTest extends OpMode {

    private Limelight3A limelight3A;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("X offset: ", llResult.getTx());
            telemetry.addData("Y offset: ", llResult.getTy());
            telemetry.addData("A offset: ", llResult.getTa());
        }
    }

}
