package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

public class Color extends GenericColor {
    public Color(int r, int g, int b, int a) {
        super(r, g, b, a);
    }

    public Color(int argb) {
        super(argb);
    }

    public Colors getColor() {
        return Colors.None; // TODO: measure color reading and return the right color
    }
}
