package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;
@Disabled
@Autonomous(name="\uD83D\uDD34 Big Triangle Auto RED", group = "AAA") // 🔴
public class BigTriangleArtefactAutoRED extends BigTriangleArtefactAuto {
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
    @Override
    public Pose convertPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),- pose.getHeading());
    }
}