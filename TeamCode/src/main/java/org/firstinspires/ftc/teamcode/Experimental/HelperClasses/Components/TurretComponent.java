package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretComponent extends MotorComponent {


    private double kV = 0.0;
    private double kA = 0.0;
    private double kStatic = 0.0;

    private double lastTarget = 0;
    private double lastTargetTime = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Robot base motion data
    private double robotAngularVelocity = 0;
    private double robotAngularAcceleration = 0;
    private double lastRobotVelocity = 0;

    public void setBaseMotion(double robotAngularVel) {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTargetTime;

        if (dt > 0) {
            this.robotAngularAcceleration = (robotAngularVel - lastRobotVelocity) / dt;
        }

        this.robotAngularVelocity = robotAngularVel;
        this.lastRobotVelocity = robotAngularVel;
        this.lastTargetTime = currentTime;
    }

    public TurretComponent setFeedforwardCoefficients(double kV, double kA, double kStatic) {
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        return this;
    }

    @Override
    public void update() {
        if (motorCurrentMode != MotorModes.Position) {
            super.update();
            return;
        }


        double currentPos = mainMotor.getCurrentPosition() / resolution;
        double pdOutput = pidControllerForPosition.calculate(target, currentPos);

        double currentTime = timer.seconds();
        double targetVelocity = (target - lastTarget) / (currentTime - lastTargetTime);
        lastTarget = target;

        double feedforward = (targetVelocity * kV) + (robotAngularVelocity * kV);

        feedforward += (robotAngularAcceleration * kA);

        double frictionComp = Math.signum(pdOutput + feedforward) * kStatic;

        double finalPower = pdOutput + feedforward + frictionComp;

        for (DcMotorEx motor : motorMap.values()) {
            motor.setPower(finalPower);
        }
    }
}