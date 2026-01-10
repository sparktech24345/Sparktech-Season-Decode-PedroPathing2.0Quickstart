package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeComponents;
import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeStates;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

@TeleOp(name = "Main TeleOp Red", group = "AAA")
public class MainTeleopRed extends MainTeleOPBlue {

    @Override
    public void teamSensitiveStuff(){
        teamPipeline = 1;
        if(targetY > 0){
            targetY = -targetY;
        }
        if(usedTargetY > 0){
            farZoneCameraAdder = - farZoneCameraAdder;
            targetYCenter = -targetYCenter;
            usedTargetY = -usedTargetY;
            targetYLeftPanel = -targetYLeftPanel;
            targetYRightPanel = -targetYRightPanel;
        }
        currentTeamColor = TeamColor.Red; // (117, 26, 90)
        farPark = new Pose(120, -14, Math.toRadians(-90));
    }
}
