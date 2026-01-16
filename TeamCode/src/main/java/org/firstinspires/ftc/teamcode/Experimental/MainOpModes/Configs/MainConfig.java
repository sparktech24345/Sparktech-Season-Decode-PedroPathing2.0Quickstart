package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
@Config
public class MainConfig {

    public enum Configs {
        Blue,
        Red
    }

    public static int teamPipeline;

    public static double farZoneCameraAdder;
    public static double autoZoneAdderFar;
    public static double rotationForInitClsoeZone;

    public static double targetX;
    public static double targetY;

    public static double targetXAutoClose;
    public static double targetYAutoClose;

    public static double targetXCenter;
    public static double targetYCenter;

    public static double targetXRightPanel; // about +6 cuz thats a tyle
    public static double targetYRightPanel; // a 3 bias for y

    public static double targetXLeftPanel; // and ofc a 3 bias on X for that one
    public static double targetYLeftPanel; // about +6 but on y

    public static double usedTargetX;
    public static double usedTargetY;

    public static TeamColor currentTeamColor;

    public MainConfig() {}

    public MainConfig(Configs cfg) {
        switch (cfg) {
            case Blue:
                teamPipeline = 0;

                farZoneCameraAdder = 0;
                autoZoneAdderFar = 0.5;

                rotationForInitClsoeZone = 145;

                targetX = 131;
                targetY = 46;


                targetXAutoClose = 127;
                targetYAutoClose = 48;

                targetXCenter = 131;
                targetYCenter = 48;

                targetXRightPanel = 131;
                targetYRightPanel = 45;

                targetXLeftPanel = 135;
                targetYLeftPanel = 55;

                usedTargetX = 131;
                usedTargetY = 40;

                currentTeamColor = TeamColor.Blue;
                break;
            case Red:
                teamPipeline = 1;

                farZoneCameraAdder = 0;
                autoZoneAdderFar = -0.5;
                rotationForInitClsoeZone = 200;

                targetX = 131;
                targetY = -46;


                targetXAutoClose = 127;
                targetYAutoClose = -48;

                targetXCenter = 131;
                targetYCenter = -48;

                targetXRightPanel = 131;
                targetYRightPanel = -45;

                targetXLeftPanel = 135;
                targetYLeftPanel = -55;

                usedTargetX = 131;
                usedTargetY = -40;

                currentTeamColor = TeamColor.Red;
                break;
        }
    }
}