package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public enum BallColorSet_Decode {
    NoBall,
    Green,
    Purple;

    public static BallColorSet_Decode getColor(NormalizedRGBA colors) {
        return getColor(colors.red, colors.green, colors.blue);
    }
    public static BallColorSet_Decode getColor(double r, double g, double b) {
        if (b > 0.001 && g > 0.001) { //first check if there is a ball then compare to determine what it is
            if (b * 1.2 > g) return Purple;
            else return Green;
        }
        else return NoBall;

    }
    public static BallColorSet_Decode getColorForTurret(NormalizedRGBA colors) {
        return getColorForTurret(colors.red * 10000.0, colors.green * 10000.0, colors.blue * 10000.0);
    }
    public static BallColorSet_Decode getColorForTurret(double r, double g, double b) {
        if(g < 9.5) return NoBall;
        else
            if(b > g) return Purple;
        else return Green;
    }


    public static BallColorSet_Decode getColorForStorage(NormalizedRGBA colors) {
        return getColorForStorage(colors.red * 10000.0, colors.green * 10000.0, colors.blue * 10000.0);
    }
    public static BallColorSet_Decode getColorForStorage(double r, double g, double b) {
        if(r < 20) return NoBall;
        else
        if(b > g) return Purple;
        else return Green;
    }

    public static BallColorSet_Decode getCameraColor(LLResult llResult) {
        if (llResult != null) {
            double[] pythonOutputs = llResult.getPythonOutput();
            if (pythonOutputs != null) {
                if (pythonOutputs[2] <= pythonOutputs[6]) {
                    return Purple;
                } else return Green;
            }
            else return NoBall;
        } else return NoBall;
    }

}
