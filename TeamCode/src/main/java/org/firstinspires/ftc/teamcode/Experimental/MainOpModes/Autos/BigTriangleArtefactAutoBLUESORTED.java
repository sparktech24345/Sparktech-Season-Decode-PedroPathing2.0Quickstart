package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Big Triangle Auto BLUE ! SORTED !", group = "AAA")
public class BigTriangleArtefactAutoBLUESORTED extends BigTriangleArtefactAuto {
    @Override
    public void methodToOverWrite(){
        super.methodToOverWrite();
        shouldMakeSortedAuto = true;
    }
}