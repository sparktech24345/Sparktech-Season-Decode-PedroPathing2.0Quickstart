package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Autonomous(name="\uD83D\uDD34 Shared Close Auto RED", group = "BBB") // 🔴
public class SharedCloseAutoRED extends SharedAutoClose {
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
    @Override
    public Pose convertPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),- pose.getHeading());
    }
}