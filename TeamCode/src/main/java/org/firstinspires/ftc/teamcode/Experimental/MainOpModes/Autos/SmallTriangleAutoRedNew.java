package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Autonomous(group="AAA", name="NEW SMALL TRIANGLE RED")
public class SmallTriangleAutoRedNew extends SmallTriangleNew {
    @Override
//    public void teamSensitiveStuff(){
//        targetY = -targetY;
//        teamPipeline = 1;
//        currentTeamColor = TeamColor.Red;
//        rotation = 19;
//    }
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
    @Override
    public Pose convertPose(Pose pose) {
        return new Pose(pose.getX(), -pose.getY(), -pose.getHeading());
    }
}