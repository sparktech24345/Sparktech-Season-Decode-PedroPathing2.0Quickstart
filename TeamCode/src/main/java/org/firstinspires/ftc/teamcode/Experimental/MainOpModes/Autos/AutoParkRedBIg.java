package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(group="park only", name="park only red big triangle")
public class AutoParkRedBIg extends AutoParkOnly {
    @Override
    public Pose getParkPose() {
        return new Pose(5, -20, 0);
    }
    @Override
    public Pose getStartingPose() {
        return new Pose(0, 0, 0);
    }
}