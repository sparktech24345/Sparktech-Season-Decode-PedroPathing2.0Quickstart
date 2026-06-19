package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="\uD83D\uDD35 Big Triangle Auto BLUE ! SORTED ! | No Adapt |", group = "AAA") // 🔵
public class BigTriangleArtefactAutoBLUESORTEDWithoutAdapt extends BigTriangleArtefactAuto {
    @Override
    public void methodToOverWrite(){
        super.methodToOverWrite();
        shouldMakeSortedAuto = true;
        shouldSTOPAdaptSorting = true;
    }
}