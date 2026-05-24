package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.grade0VeloClose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.grade1VeloClose;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretComponent extends MotorComponent {

    private double kV = 0.0, kA = 0.0, kStatic = 0.0;
    private double lastX = 0, lastY = 0, lastAngle = 0;
    private double lastTimestamp = 0;
    private double robotAngularVel = 0;
    private double vx = 0, vy = 0;
    private double x_offset=8;
    private double lastX_cam = 0, lastY_cam = 0;
    private double lastAngle_cam = 0;

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

        double angle_cam = globalRobotPose.getHeading();
//        double x_cam = x + x_offset * Math.cos(angle_cam);
//        double y_cam = y + x_offset * Math.sin(angle_cam);

        // Convert degrees to radians first
        double angle_rad = Math.toRadians(globalRobotPose.getHeading());

        // Now use the radian value for your trig
        double x_cam = x + x_offset * Math.cos(angle_rad);
        double y_cam = y + x_offset * Math.sin(angle_rad);

//        double theta = globalRobotPose.getHeading(); // Ensure this is Radians
//        double x_cam = x + (x_offset * Math.cos(theta)) - (x_offset * Math.sin(theta));
//        double y_cam = y + (x_offset * Math.sin(theta)) + (x_offset * Math.cos(theta));

        double currentTime = timer.seconds();
        double dt = currentTime - lastTimestamp;
        if (dt <= 0) return;

        double deltaAngle = angle - lastAngle;
        while (deltaAngle > 180) deltaAngle -= 360;
        while (deltaAngle < -180) deltaAngle += 360;
        this.robotAngularVel = deltaAngle / dt;


        double deltaAngle_cam = angle_cam - lastAngle_cam;
        while (deltaAngle_cam > 180) deltaAngle_cam -= 360;
        while (deltaAngle_cam < -180) deltaAngle_cam += 360;
//        this.robotAngularVel = deltaAngle / dt;

        this.vx = (x - lastX) / dt;
        this.vy = (y - lastY) / dt;

        this.lastX = x;
        this.lastY = y;
        this.lastX_cam = x_cam;
        this.lastY_cam = y_cam;
        this.lastAngle = angle;
        this.lastAngle_cam = angle_cam;
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
    public double calculateTurretCamera(double targetX, double targetY, double lookaheadSeconds) {
        // Use smoothed velocities to predict FUTURE position
        double xc = globalRobotPose.getX();
        double yc = globalRobotPose.getY();
        double angle_cam = globalRobotPose.getHeading();
        double x_cam = xc + x_offset * Math.cos(angle_cam);
        double y_cam = yc + x_offset * Math.sin(angle_cam);

        double dx = targetX - x_cam;
        double dy = targetY - y_cam;

        double worldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double relative = worldAngle;

        while (relative > 180) relative -= 360;
        while (relative < -180) relative += 360;

        return relative;
    }

    public double calculateLookaheadTarget_Camera(double targetX, double targetY) {
        double predX_cam = lastX_cam;
        double predY_cam = lastY_cam;


        double dx_cam = targetX - predX_cam;
        double dy_cam = targetY - predY_cam;

        double worldAngle_cam = Math.toDegrees(Math.atan2(dy_cam, dx_cam));
        double relative = worldAngle_cam - lastAngle;

        while (relative > 180) relative -= 360;
        while (relative < -180) relative += 360;

        return relative;
    }

    @Override
    public void update() {
        if (motorCurrentMode != MotorModes.Position || target == 0) {
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