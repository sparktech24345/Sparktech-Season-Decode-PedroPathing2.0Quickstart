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

        // if (Math.signum(error) != Math.signum(integral)) integral = 0;

        // Integral term (accumulated error)
        integral += error * deltaTime;

        // Derivative term (rate of change of error)
        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        lastError = error;

        // PID output
        double output = (kp * error) + (ki * integral) + (kd * derivative);

        // Clamp to motor power limits
        //output = clamp(output, -1.0, 1.0); //Not needed and detrimental for tests

        return output;
    }

    // target position, current position
    public double calculateSpecial(double target, double current,double kv,double ks) {
        long now = System.nanoTime();
        double deltaTime = (now - lastTime);  // in nanoseconds
        lastTime = now;

        // target velocity
        double targetVelocity = (target - current) / 10;

        //vel
        double velocity = (deltaTime > 0) ? (current - lastPos) / deltaTime : 0;
        lastPos = current;

        // derivata la vel, adica acceleratia
        double derivative = (deltaTime > 0) ? (velocity - lastVelocity) / deltaTime : 0;
        lastVelocity = velocity;

        double errorVelocity = targetVelocity - velocity;

        // PID output
        double output = ks + (kp * velocity) + (kd * errorVelocity);

        // Clamp to motor power limits
        //output = clamp(output, -1.0, 1.0); //Not needed and detrimental for tests

        return output;
    }
}
