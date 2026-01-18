package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretComponent extends MotorComponent {

    // New Feedforward coefficients
    private double kV = 0.0; // Power needed to maintain steady velocity
    private double kA = 0.0; // Power needed to overcome inertia (acceleration)
    private double kStatic = 0.0; // Power needed to break static friction

    private double lastTarget = 0;
    private double lastTargetTime = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Robot base motion data
    private double robotAngularVelocity = 0;
    private double robotAngularAcceleration = 0;
    private double lastRobotVelocity = 0;

    /**
     * Call this in your main loop to feed the robot's motion data into the turret
     * @param robotAngularVel The current angular velocity of the robot base (deg/s or rad/s)
     */
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

        // 1. Calculate Standard PD Output
        double currentPos = mainMotor.getCurrentPosition() / resolution;
        double pdOutput = pidControllerForPosition.calculate(target, currentPos);

        // 2. Calculate Target Velocity (how fast the target is moving relative to the robot)
        double currentTime = timer.seconds();
        double targetVelocity = (target - lastTarget) / (currentTime - lastTargetTime);
        lastTarget = target;
        // lastTargetTime is updated in setBaseMotion or here if not using it

        // 3. BASE COMPENSATION LOGIC
        // If the robot turns right, the turret must turn left at the same speed to stay locked.
        // We add the robot's velocity to the feedforward to "cancel" the motion.
        double feedforward = (targetVelocity * kV) + (robotAngularVelocity * kV);

        // 4. Acceleration Feedforward (to stop the "falling behind" at the start)
        feedforward += (robotAngularAcceleration * kA);

        // 5. Static Friction (kStatic)
        double frictionComp = Math.signum(pdOutput + feedforward) * kStatic;

        double finalPower = pdOutput + feedforward + frictionComp;

        // Apply to motors
        for (DcMotorEx motor : motorMap.values()) {
            motor.setPower(finalPower);
        }
    }
}