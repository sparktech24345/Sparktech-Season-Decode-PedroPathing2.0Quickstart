package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="\uD83D\uDD34 Big Triangle Auto REEEDDD Without 3rd row", group = "BAA") // 🔴
public class BigTriangleArtefactAutoREDWithout3rdRow extends BigTriangleArtefactAutoRED {
    @Override
    public void methodToOverWrite(){
        super.methodToOverWrite();
        shouldMakeAutoWithout3rdRow = true;
    }
}