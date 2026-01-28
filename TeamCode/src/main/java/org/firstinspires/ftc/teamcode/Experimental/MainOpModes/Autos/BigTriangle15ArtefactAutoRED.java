package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;
@Disabled
@Autonomous(name="Big Triangle 15 Artefact Auto RED", group = "BBB")
public class BigTriangle15ArtefactAutoRED extends BigTriangle15ArtefactAutoBLUE {
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
    @Override
    public Pose convertPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),- pose.getHeading());
    }
}