package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public enum BallColorSet_Decode {
    NoBall,
    Green,
    Purple;

    public static BallColorSet_Decode getColor(NormalizedRGBA colors) {
        return getColor(colors.red, colors.green, colors.blue);
    }
    public static BallColorSet_Decode getColor(double r, double g, double b) {
        if(b > 0.001 && g > 0.001){ //first check if there is a ball then compare to determine what it is
            if(b*1.2 > g) return Purple;
            else return Green;
        }
        else return NoBall;

    }
}
