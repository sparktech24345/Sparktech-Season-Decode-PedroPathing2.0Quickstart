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
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

@TeleOp(name = "OLD Main TeleOP Red", group = "BBB")
public class OldMainTeleopRed extends OldMainTeleOPBlue {
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
}
