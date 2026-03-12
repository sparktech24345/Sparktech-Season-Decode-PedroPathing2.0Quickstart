package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Big Triangle Auto REEEDDD Without 3rd row", group = "BAA")
public class BigTriangleArtefactAutoREDWithout3rdRow extends BigTriangleArtefactAutoRED {
    @Override
    public void methodToOverWrite(){
        super.methodToOverWrite();
        shouldMakeAutoWithout3rdRow = true;
    }
}