package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

public class PIDcontroller {
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double integral = 0;
    private double lastError = 0;
    private double lastVelocity = 0;
    private double lastPos = 0;
    private long lastTime = System.currentTimeMillis();

    public PIDcontroller(double p, double i, double d) {
        setConstants(p, i, d);
    }

    public PIDcontroller() {}

    public void setConstants(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
    }

    public double getIntegralSum() {
        return integral;
    }

    public void setIntegralSum(double sum) {
        integral = sum;
    }

    public double getKi() {
        return ki;
    }
    public double getKp() {
        return kp;
    }
    public double getKd() {
        return kd;
    }

    public double calculate(double target, double current) {
        long now = System.currentTimeMillis();
        double deltaTime = (now - lastTime) / 1000.0;  // in seconds
        lastTime = now;

        double error = target - current;

        integral += error * deltaTime;

        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        lastError = error;

        double output = (kp * error) + (ki * integral) + (kd * derivative);

        return output;
    }
}
