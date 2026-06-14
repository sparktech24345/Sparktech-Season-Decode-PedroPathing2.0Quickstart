package org.firstinspires.ftc.teamcode.Experimental.HelperClasses; // Change to your actual package

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;

public class CameraMath {

    public static class SimplePose {
        public double x;
        public double y;
        public double heading; // In degrees

        public SimplePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    /**
     * METHOD 1: Calculates the target relative angle for the servo (CCW standard format).
     */
    public static double calculateCameraAngle(
            double targetX, double targetY,
            SimplePose robotPose,
            double camOffsetX, double camOffsetY) {

        double robotX = robotPose.x;
        double robotY = robotPose.y;
        double robotAngleDeg = robotPose.heading;
        double robotRad = Math.toRadians(robotAngleDeg);

        double rotatedCamX = camOffsetX * Math.cos(robotRad) - camOffsetY * Math.sin(robotRad);
        double rotatedCamY = camOffsetX * Math.sin(robotRad) + camOffsetY * Math.cos(robotRad);

        double absoluteCamX = robotX + rotatedCamX;
        double absoluteCamY = robotY + rotatedCamY;

        double deltaX = targetX - absoluteCamX;
        double deltaY = targetY - absoluteCamY;

        double absoluteTargetAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double relativeCamAngle = absoluteTargetAngleDeg - robotAngleDeg;

        return normalizeDegrees(relativeCamAngle);
    }

    /**
     * METHOD 2: Converts relative angle to your hardware servo positional value.
     */
    public static double convertCamAngleToServoValue(double relativeAngle) {
        double servoValue = 0.49 + (relativeAngle * (0.29 / 90.0));
        if (servoValue < 0.0) servoValue = 0.0;
        if (servoValue > 1.0) servoValue = 1.0;
        return servoValue * 360.0;
    }

    /**
     * METHOD 3 (FIXED): Calculates the absolute global field orientation of the camera lens for MT2.
     * Properly compensates for your servo's inverted physical rotation direction.
     */
    public static double calculateGlobalCameraOrientationForMT2(double robotHeadingDeg, double servoAngle) {
        double servoValue = servoAngle / 360.0;

        // This decodes your servo position back into degrees from center (Clockwise positive)
        double angleFromCenterCW = (servoValue - 0.49) * (90.0 / 0.29);

        // Convert to standard Counter-Clockwise degrees to match the robot IMU orientation
        double relativeTurretAngleCCW = -angleFromCenterCW;

        // Absolute field orientation of the camera lens = Robot Heading + Turret Offset
        double absoluteLensAngle = robotHeadingDeg + relativeTurretAngleCCW;

        // Apply your custom 180-degree field orientation tracking flip
        return normalizeDegrees(absoluteLensAngle - 180.0);
    }

    /**
     * METHOD 4 (FIXED): Reconstructs absolute field positions.
     * Rotates your hardware offsets using the physical chassis heading rather than lens heading.
     */
    public static SimplePose relocalizeRobot(
            double rawCamX, double rawCamY, double camDegrees, TeamColor color,
            double camOffsetX, double camOffsetY, double servoAngle, int numberOfTags) {

        // Structural base field offsets
        double xStart = 0;
        double yStart = 0;
        if (color == TeamColor.Blue) {
            xStart = -1.57;
            yStart = -0.445; // always presume 2 tag whatever
        } else {
            xStart = -1.57;
            yStart = 0.445;
        }

        double servoValue = servoAngle / 360.0;
        double angleFromCenterCW = (servoValue - 0.49) * (90.0 / 0.29);

        // Calculate absolute chassis position heading using the servo offset angle
        double robotAngle = camDegrees + angleFromCenterCW + 180.0;
        robotAngle = normalizeDegrees(robotAngle);

        // Scale Limelight data dimensions out to inches
        double scaleFactor = 39.3701;
        double cameraWorldX = -scaleFactor * (rawCamX + xStart);
        double cameraWorldY = -scaleFactor * (rawCamY - yStart);

        // Rotate the hardware mounting pivot offset relative to the ROBOT chassis orientation
        double robotRad = Math.toRadians(robotAngle);
        double rotatedOffsetX = camOffsetX * Math.cos(robotRad) - camOffsetY * Math.sin(robotRad);
        double rotatedOffsetY = camOffsetX * Math.sin(robotRad) + camOffsetY * Math.cos(robotRad);

        // Translate tracking from the lens position back to the absolute center of the chassis
        double robotX = cameraWorldX - rotatedOffsetX;
        double robotY = cameraWorldY - rotatedOffsetY;

        return new SimplePose(robotX, robotY, robotAngle);
    }

    private static double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}