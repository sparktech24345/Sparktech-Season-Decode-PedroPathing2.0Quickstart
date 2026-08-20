package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightComponent extends ThreadComponent {

    protected Limelight3A limelight;

    // Volatile primitive fields for zero-latency main thread reads
    protected volatile LLResult llResult = null;
    protected volatile boolean hasTarget = false;


    protected volatile boolean shouldSwapPipeline = false;
    protected volatile int currentPipeline = 0;

    public LimelightComponent addLimelight(String hardwareMapName) {
        limelight = hardwareMap.get(Limelight3A.class, hardwareMapName);
        limelight.pipelineSwitch(0);
        limelight.start();
        return this;
    }
    public LimelightComponent pipelineSwitch(int pipeline){

        return this;
    };

    @Override
    protected void runAsync() {
        if (limelight == null) return;
        if(shouldSwapPipeline){
            shouldSwapPipeline = false;
            limelight.pipelineSwitch(currentPipeline);
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            this.llResult = limelight.getLatestResult();
            this.hasTarget = true;
        } else {
            this.hasTarget = false;
        }
    }

    // Main thread getters
    public boolean hasTarget() { return hasTarget; }
    public LLResult getLatestResult() {
        return this.llResult;
    }
}