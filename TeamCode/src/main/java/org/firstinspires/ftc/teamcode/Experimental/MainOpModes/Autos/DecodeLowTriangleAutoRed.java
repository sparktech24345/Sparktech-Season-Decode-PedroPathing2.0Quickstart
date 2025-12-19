package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeComponents;
import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeStates;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

@Autonomous(name = "Auto Small Triangle Red", group = "Main")
public class DecodeLowTriangleAutoRed extends DecodeLowTriangleBlueAuto {
    @Override
    public Pose ModifyPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),pose.getHeading());
    }
}
