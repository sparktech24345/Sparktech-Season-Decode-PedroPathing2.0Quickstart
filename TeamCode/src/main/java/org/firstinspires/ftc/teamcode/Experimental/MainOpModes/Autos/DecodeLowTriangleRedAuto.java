package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.farZoneCameraAdder;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;

@Disabled
@Autonomous(name = "Auto Small Triangle Red", group = "Main")
public class DecodeLowTriangleRedAuto extends DecodeLowTriangleBlueAuto {
    @Override
    public Pose ModifyPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),pose.getHeading());
    }
    @Override
    public void FixTeamStuff(){
        // init
        currentTeamColor = TeamColor.Red;
        teamPipeline = 1;

        farZoneCameraAdder =  - farZoneCameraAdder;
        targetY =  - targetY;
    }
}
