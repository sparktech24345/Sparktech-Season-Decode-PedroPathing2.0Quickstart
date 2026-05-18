package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.pedropathing.geometry.Pose;

public class Math {
    static {
       System.loadLibrary("ftcrobotcontroller");
    }

    public native static double sqrt(double a);
    public native static double max(double a1, double a2);
    public native static double min(double a1, double a2);
    public native static double abs(double a);
    public native static boolean eval(double a);
    public native static double eval(boolean a);
    public native static boolean triggerEval(double t);
    public native static double calculateDistance(Pose pose1, Pose pose2, boolean convertToMeters);
    public native static double calculateDistanceNonZero(Pose pose1, Pose pose2, boolean convertToMeters);

    public native static double degreesToOuttakeTurretServo(double degrees);
    public native static double voltageMultiplierForMotor(double voltage);
}
