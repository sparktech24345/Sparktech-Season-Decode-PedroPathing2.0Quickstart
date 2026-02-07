package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretComponent extends MotorComponent {

    // Feedforward Gains
    private double kV = 0.0; // Velocity gain
    private double kA = 0.0; // Acceleration gain
    private double kStatic = 0.0; // Static friction/stiction gain

    // Tracking for physics calculations
    private double lastX = 0, lastY = 0, lastAngle = 0;
    private double lastTimestamp = 0;
    private double robotAngularVel = 0;
    private double vx = 0, vy = 0;

    private final ElapsedTime timer = new ElapsedTime();
    public TurretComponent setFeedforwardCoefficients(double kV, double kA, double kStatic) {
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        return this;
    }

    public void updateRobotPose(Pose robotPose) {

        double x = robotPose.getX();
        double y = robotPose.getY();
        double angle = Math.toDegrees(robotPose.getHeading());


        double currentTime = timer.seconds();
        double dt = currentTime - lastTimestamp;
        if (dt <= 0) return;

        // Calculate Angular Velocity with wrap-around check
        double deltaAngle = angle - lastAngle;
        while (deltaAngle > 180) deltaAngle -= 360;
        while (deltaAngle < -180) deltaAngle += 360;
        this.robotAngularVel = deltaAngle / dt;

        // Calculate Linear Velocity
        this.vx = (x - lastX) / dt;
        this.vy = (y - lastY) / dt;

        this.lastX = x;
        this.lastY = y;
        this.lastAngle = angle;
        this.lastTimestamp = currentTime;
    }

    public double calculateLookaheadTarget(double targetX, double targetY, double lookaheadSeconds) {
        double predX = lastX + (vx * lookaheadSeconds);
        double predY = lastY + (vy * lookaheadSeconds);

        double dx = targetX - predX;
        double dy = targetY - predY;

        double predictedAngleWorld = Math.toDegrees(Math.atan2(dy, dx));

        // Target relative to robot heading
        double relativeAngle = predictedAngleWorld - lastAngle;
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        return relativeAngle;
    }

    @Override
    public void update() {
        // If we aren't in position mode, just act like a normal motor
        if (motorCurrentMode != MotorModes.Position) {
            super.update();
            return;
        }

        // 1. Get PD Output
        double currentPos = mainMotor.getCurrentPosition() / resolution;
        double pdOutput = pidControllerForPosition.calculate(target, currentPos);

        // 2. Add Base Compensation (FF)
        // This pushes the motor to cancel out the robot's rotation
        double feedforward = (robotAngularVel * kV);

        // 3. Final Output Assembly
        double totalOutput = pdOutput + feedforward;

        // 4. Add Static Friction (only if we are actually trying to move)
        if (Math.abs(target-currentPos) > 0.25) {
            totalOutput += Math.signum(totalOutput) * kStatic;
        }
        //else totalOutput = 0;

        double finalPower = Range.clip(totalOutput, -1, 1);

        for (DcMotorEx motor : motorMap.values()) {
            motor.setPower(finalPower);
        }
    }
}