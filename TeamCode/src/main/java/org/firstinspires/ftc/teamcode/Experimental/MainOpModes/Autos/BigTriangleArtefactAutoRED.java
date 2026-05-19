package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.Configuration;

//@Autonomous(name="Big Triangle Auto RED", group = "AAA")
public class BigTriangleArtefactAutoRED extends BigTriangleArtefactAuto {
    public void makeConfig(){
        //cfg = Configuration.getConfig("red");
    }
    @Override
    public Pose convertPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),- pose.getHeading());
    }
}