package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

public abstract class GenericColor {
    protected int r = 0;
    protected int g = 0;
    protected int b = 0;
    protected int a = 0;

    public GenericColor(int r, int g, int b, int a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    public GenericColor(int argb) {
        a = (argb >> 24) & 0xFF;
        r = (argb >> 16) & 0xFF;
        g = (argb >> 8) & 0xFF;
        b = argb & 0xFF;
    }

    public int r() {
        return r;
    }

    public int g() {
        return g;
    }

    public int b() {
        return b;
    }

    public int a() {
        return a;
    }

    public enum Colors {
        White,
        Black,
        None,
        Blue,
        Red,
        Green,
        Purple,
        Cyan,
        Yellow
    }

    abstract Colors getColor();
}
