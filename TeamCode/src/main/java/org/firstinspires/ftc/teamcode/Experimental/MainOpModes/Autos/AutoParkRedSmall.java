package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;

@Autonomous(group="park only", name="park only red small triangle")
public class AutoParkRedSmall extends AutoParkOnly {
    @Override
    public Pose getParkPose() {
        return new Pose(15, -15, 0);
    }

    @Override
    public Pose getStartingPose() {
        return new Pose(0, 0, 0);
    }
}