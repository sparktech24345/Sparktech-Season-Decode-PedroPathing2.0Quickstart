package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="\uD83D\uDD35 Big Triangle Auto BLUE Without 3rd row", group = "BAA") // 🔵
public class BigTriangleArtefactAutoBLUEWithout3rdRow extends BigTriangleArtefactAuto {
    @Override
    public void methodToOverWrite(){
        super.methodToOverWrite();
        shouldMakeAutoWithout3rdRow = true;
    }
}