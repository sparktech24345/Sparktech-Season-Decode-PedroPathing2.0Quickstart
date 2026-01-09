package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;

public class RedConfig extends MainConfig {
    public RedConfig() {
        MainConfig.teamPipeline = 1;
        MainConfig.farZoneCameraAdder = 1;
        MainConfig.targetY = -46;
        MainConfig.targetYCenter = -46;
        MainConfig.usedTargetY = -46;
        MainConfig.targetYLeftPanel = -55;
        MainConfig.targetYRightPanel = -48;
        MainConfig.currentTeamColor = TeamColor.Red;
    }
}
