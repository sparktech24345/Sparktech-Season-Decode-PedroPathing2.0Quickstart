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
    public static double autoVelAdder;
    public static double rotationForInitClsoeZone;
    public static double rotationForInitSmallTriangle;

    public static double targetX;
    public static double targetY;

    public static double targetXAutoClose;
    public static double targetYAutoClose;

    public static double targetXCenter;
    public static double targetYCenter;

    public static double targetXRightPanel;
    public static double targetYRightPanel;

    public static double targetXLeftPanel;
    public static double targetYLeftPanel;

    public static double usedTargetX;
    public static double usedTargetY;

    public static double targetForCameraX;
    public static double targetForCameraY;

    public static TeamColor currentTeamColor;

    public MainConfig() {}

    public MainConfig(Configs cfg) {
        switch (cfg) {
            case Blue:
                teamPipeline = 0;

                farZoneCameraAdder = 0;
                autoZoneAdderFar = 0;

                targetForCameraX = 16;
                targetForCameraY = 50;

                //autoVelAdder = -20;

                rotationForInitClsoeZone = 245;
                rotationForInitSmallTriangle = 90;

                targetX = 130;
                targetY = 53;

                targetXAutoClose = 127;
                targetYAutoClose = 48;

                targetXCenter = 128; // 130
                targetYCenter = 53;

                targetXRightPanel = 127;
                targetYRightPanel = 48;

                targetXLeftPanel = 129;
                targetYLeftPanel = 55;

                usedTargetX = 131;
                usedTargetY = 40;

                currentTeamColor = TeamColor.Blue;
                break;
            case Red:
                teamPipeline = 1;

                farZoneCameraAdder = 0;
                autoZoneAdderFar = 0;


                targetForCameraX = 16;
                targetForCameraY = -50;

                //autoVelAdder = -20;

                rotationForInitClsoeZone = 115;
                rotationForInitSmallTriangle = 270;

                targetX = 130;
                targetY = -53;

                targetXAutoClose = 127;
                targetYAutoClose = -43;

                targetXCenter = 128; // 130
                targetYCenter = -53;

                targetXRightPanel = 127;
                targetYRightPanel = -43; //

                targetXLeftPanel = 125;
                targetYLeftPanel = -55;

                usedTargetX = 131;
                usedTargetY = -40;

                currentTeamColor = TeamColor.Red;
                break;
        }
    }
}