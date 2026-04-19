package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Big Triangle Auto RED ! SORTED !", group = "AAA")
public class BigTriangleArtefactAutoREDSORTED extends BigTriangleArtefactAutoRED {
    @Override
    public void methodToOverWrite() {
        super.methodToOverWrite();
        shouldMakeSortedAuto = true;
    }
}