package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public enum ColorSet_ITD {
    Undefined,
    Red,
    Blue,
    Yellow;

    public static boolean validateSample(ColorSet_ITD current, boolean yellowValid) {
        if (yellowValid && current == Yellow) return true;
        return current == currentTeamITD;
    }

    public static ColorSet_ITD getColor(NormalizedRGBA colors) {
        return getColor(colors.red, colors.green, colors.blue);
    }
    public static ColorSet_ITD getColor(double r, double g, double b) {
        if (r < 0.006 && b < 0.004) return Undefined;
        if (r > g && r > b) return Red;
        if (b > g && b > r) return Blue;
        if (g > r && g > b) return Yellow;
        return Undefined;
    }
}
