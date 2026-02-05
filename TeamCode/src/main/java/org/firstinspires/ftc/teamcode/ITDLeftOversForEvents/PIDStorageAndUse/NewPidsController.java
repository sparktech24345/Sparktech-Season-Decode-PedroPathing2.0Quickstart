package org.firstinspires.ftc.teamcode.ITDLeftOversForEvents.PIDStorageAndUse;

public class NewPidsController {

    public static double integral = 0;
    public static double lastError = 0;
    public static long lastTime = System.currentTimeMillis();
    public static double integralUppy = 0;
    public static double lastErrorUppy= 0;
    public static long lastTimeUppy = System.currentTimeMillis();
    public static double kp = 0.009;
    public static double ki = 0.06691449814126393;
    public static double kd = 0.000302625;
    public static double kpUppy = 0.0105;
    public static double kiUppy = 0.06691449814126393;
    public static double kdUppy = 0.000112875;


    public static double pidControllerOuttake(double targetUppy, double currentUppy) {
        long nowUppy = System.currentTimeMillis();
        double deltaTimeUppy = (nowUppy - lastTimeUppy) / 1000.0;
        lastTimeUppy = nowUppy;

        double errorUppy = targetUppy - currentUppy;

        integralUppy += errorUppy * deltaTimeUppy;

        double derivativeUppy = (deltaTimeUppy > 0) ? (errorUppy - lastErrorUppy) / deltaTimeUppy : 0;
        lastErrorUppy = errorUppy;

        double outputUppy = (kpUppy * errorUppy)  + (kdUppy * derivativeUppy);

        outputUppy = Math.max(-1.0, Math.min(1.0, outputUppy));

        return outputUppy;
    }
    public static double pidControllerIntake(double target, double current) {
        long now = System.currentTimeMillis();
        double deltaTime = (now - lastTime) / 1000.0;
        lastTime = now;

        double error = target - current;

        integral += error * deltaTime;

        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        lastError = error;

        double output = (kp * error) + (kd * derivative);

        output = Math.max(-1.0, Math.min(1.0, output));

        return output;
    }

}
