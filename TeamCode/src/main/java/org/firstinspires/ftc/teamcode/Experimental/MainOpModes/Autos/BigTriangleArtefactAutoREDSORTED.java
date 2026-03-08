package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Autonomous(name="Big Triangle Auto RED ! SORTED !", group = "AAA")
public class BigTriangleArtefactAutoREDSORTED extends BigTriangleArtefactAutoRED {
    @Override
    public void methodToOverWrite(){
        super.methodToOverWrite();
        shouldMakeSortedAuto = true;
    }
}