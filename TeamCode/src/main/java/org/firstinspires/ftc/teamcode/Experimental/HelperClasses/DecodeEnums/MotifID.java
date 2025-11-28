package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public enum MotifID {
    Null,
    G_P_P, // id21
    P_G_P, // id22
    P_P_G; //id23
    public static MotifID getMotif(Limelight3A limelight3A){
        limelight3A.pipelineSwitch(2);
        LLResult llResult = limelight3A.getLatestResult();
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{21, 22, 23}); // Only track these tag IDs
        if (llResult.){

        }
    }
}
