package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.grade0VeloClose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.grade1VeloClose;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretComponent extends MotorComponent {

    private double kV = 0.0, kA = 0.0, kStatic = 0.0;
    private double lastX = 0, lastY = 0, lastAngle = 0;
    private double lastTimestamp = 0;
    private double robotAngularVel = 0;
    private double vx = 0, vy = 0;

    private final ElapsedTime timer = new ElapsedTime();

    // The time the ball spends in the air.
    // Since you observed ~1s, we use this to offset robot velocity drift.
    private double ballTimeInAir = 1.0;

    public TurretComponent setFeedforwardCoefficients(double kV, double kA, double kStatic) {
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        return this;
    }

    public void setBallTimeInAir(double seconds) {
        this.ballTimeInAir = seconds;
    }

    public void updateRobotPose(Pose robotPose) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        double angle = Math.toDegrees(robotPose.getHeading());

        double currentTime = timer.seconds();
        double dt = currentTime - lastTimestamp;
        if (dt <= 0) return;

        double deltaAngle = angle - lastAngle;
        while (deltaAngle > 180) deltaAngle -= 360;
        while (deltaAngle < -180) deltaAngle += 360;
        this.robotAngularVel = deltaAngle / dt;

        this.vx = (x - lastX) / dt;
        this.vy = (y - lastY) / dt;

        this.lastX = x;
        this.lastY = y;
        this.lastAngle = angle;
        this.lastTimestamp = currentTime;
    }

    /**
     * Calculates target angle AND distance for the flywheel
     */
    public double calculateLookaheadTarget(double targetX, double targetY, double lookaheadSeconds) {
        // Use smoothed velocities to predict FUTURE position
        double predX = lastX + (vx * lookaheadSeconds);
        double predY = lastY + (vy * lookaheadSeconds);

        double dx = targetX - predX;
        double dy = targetY - predY;

        double worldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double relative = worldAngle - lastAngle;

        while (relative > 180) relative -= 360;
        while (relative < -180) relative += 360;

        return relative;
    }

    /**
     * Your provided velocity function integrated into the component
     * @return The required RPM/Velocity for your flywheel
     */
    public double getTargetFlywheelVelocity(double currentDistanceToTarget) {
        return distanceToVelocityFunction(currentDistanceToTarget);
    }

    @Override
    public void update() {
        if (motorCurrentMode != MotorModes.Position) {
            super.update();
            return;
        }

        double currentPos = mainMotor.getCurrentPosition() / resolution;
        double pdOutput = pidControllerForPosition.calculate(target, currentPos);

        // Feedforward to fight the 70ms loop delay for heading
        double feedforward = (robotAngularVel * kV);

        double totalOutput = pdOutput + feedforward;

        // Static friction kick
        if (Math.abs(target - currentPos) > 0.25) {
            totalOutput += Math.signum(totalOutput) * kStatic;
        }

        double finalPower = Range.clip(totalOutput, -1, 1);
        for (DcMotorEx motor : motorMap.values()) {
            motor.setPower(finalPower);
        }
    }

    // Getters for your distance function logic
    public double getVx() { return vx; }
    public double getVy() { return vy; }
}