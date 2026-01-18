package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Autonomous(group="BBB", name="Big Triangle Auto RED 12 Balls")
public class BigTriangle12ArtefactPushAutoRED extends BigTriangle12ArtefactPushAuto {
    @Override
//    public void teamSensitiveStuff(){
//        targetY = -targetY;
//        teamPipeline = 1;
//        currentTeamColor = TeamColor.Red;
//        rotation = - rotation;
//        rotationForInit  = 210;
//    }
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
    @Override
    public Pose convertPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),- pose.getHeading());
    }
}