package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeComponents;
import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeStates;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

@TeleOp(name = "Main TeleOp Red", group = "Main")
public class MainTeleopRed extends MainTeleOPBlue {

    @Override
    public void teamSensitiveStuff(){
        teamPipeline = 1;
        farZoneCameraAdder = - farZoneCameraAdder;
        targetY = -targetY;
        targetYCenter = -targetYCenter;
        usedTargetY = -usedTargetY;
        targetYLeftPanel = -targetYLeftPanel;
        targetYRightPanel = -targetYRightPanel;
        currentTeamColor = TeamColor.Red;
    }
}
