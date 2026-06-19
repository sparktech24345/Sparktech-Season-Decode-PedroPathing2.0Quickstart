package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="\uD83D\uDD34 Big Triangle Auto RED ! SORTED ! | No Adapt |", group = "AAA")// 🔴
public class BigTriangleArtefactAutoREDSORTEDWithoutAdapt extends BigTriangleArtefactAutoRED {
    @Override
    public void methodToOverWrite(){
        super.methodToOverWrite();
        shouldMakeSortedAuto = true;
        shouldSTOPAdaptSorting = true;
    }
}