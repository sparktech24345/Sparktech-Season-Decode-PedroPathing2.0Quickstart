package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;

public class MainConfig {

    public enum Configs {
        Blue,
        Red
    }

    public int teamPipeline;

    public double farZoneCameraAdder;

    public double targetX;
    public double targetY;

    public double targetXCenter;
    public double targetYCenter;

    public double targetXRightPanel; // about +6 cuz thats a tyle
    public double targetYRightPanel; // a 3 bias for y

    public double targetXLeftPanel; // and ofc a 3 bias on X for that one
    public double targetYLeftPanel; // about +6 but on y

    public double usedTargetX;
    public double usedTargetY;

    public TeamColor currentTeamColor;

    public MainConfig() {}

    public MainConfig(Configs cfg) {
        switch (cfg) {
            case Blue:
                teamPipeline = 0;

                farZoneCameraAdder = -1;

                targetX = 125;
                targetY = 46;

                targetXCenter = 125;
                targetYCenter = 46;

                targetXRightPanel = 131;
                targetYRightPanel = 48;

                targetXLeftPanel = 126;
                targetYLeftPanel = 55;

                usedTargetX = 125;
                usedTargetY = 46;

                currentTeamColor = TeamColor.Blue;
                break;
            case Red:
                teamPipeline = 1;

                farZoneCameraAdder = 1;

                targetX = 125;
                targetY = -46;

                targetXCenter = 125;
                targetYCenter = -46;

                targetXRightPanel = 131;
                targetYRightPanel = -48;

                targetXLeftPanel = 126;
                targetYLeftPanel = -55;

                usedTargetX = 125;
                usedTargetY = -46;

                currentTeamColor = TeamColor.Red;
                break;
        }
    }
}