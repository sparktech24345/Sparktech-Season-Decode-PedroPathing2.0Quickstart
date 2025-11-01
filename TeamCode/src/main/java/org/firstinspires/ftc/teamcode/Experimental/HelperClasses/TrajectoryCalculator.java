package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

/**
 * Calculates the minimum initial velocity (v0) and the corresponding launch angle (theta)
 * required to clear a specific obstacle, based on the projectile motion equation.
 *
 * NOTE: This solution ignores air resistance.
 */
@Config
public class TrajectoryCalculator {

    public static double trajA = 0.1;
    public static double trajB = 0;
    public static double wallHeigt = 1.0;

    public class TrajectoryResult {
        public TrajectoryResult(double minInitialVelocity, double optimalAngleDegrees) {
            this.minInitialVelocity = minInitialVelocity;
            this.optimalAngleDegrees = optimalAngleDegrees;
        }

        public double getMinInitialVelocity() {
            return minInitialVelocity;
        }

        public double getOptimalAngleDegrees() {
            return optimalAngleDegrees;
        }

        public double minInitialVelocity;
        public double optimalAngleDegrees;
    }

    // --- CONSTANTS (All in METERS and M/S^2) ---
    private static double G_ACCELERATION = 9.81; // Gravity magnitude (m/s^2)
    private static double Y0_INITIAL_HEIGHT = 0.310; // Initial height (m)
    private static double H_WALL_HEIGHT = 1.00; // Wall height (m)
    private static double C_CLEARANCE_MARGIN = 0.15; // Clearance (m)
    private static double Y_TARGET = H_WALL_HEIGHT + C_CLEARANCE_MARGIN; // 1.15 m
    private static double Y_DIFFERENCE = Y_TARGET - Y0_INITIAL_HEIGHT; // 0.84 m
    final double RPM_CONVERSION_FACTOR = (60 / (2 * Math.PI * 0.036)) * (34.0 / 22.0) / 6;

    // !!! NEW CONSTANT for MOTOR SCALING !!!
    // Set this based on your mechanism's maximum experimental launch speed (m/s).
    // This example uses a placeholder of 15 m/s, you MUST tune this.
    public static double MAX_LAUNCH_VELOCITY_MS = 15.0;

    private static double minAngleDeg = 55;
    private static double maxAngleDeg = 70;
    public double calculateDistance(Pose pose1, Pose pose2, boolean convertToMeters) {

        double dx = pose2.getX() - pose1.getX();
        double dy = pose2.getY() - pose1.getY();

        double distance = Math.sqrt(dx * dx + dy * dy);

        if (convertToMeters) {
            // Assuming your Pose coordinates are in INCHES (FTC standard)
            distance *= 0.0254; // inches to meters
        }

        if(distance == 0) return  0.001;

        return distance;
    }

    public double calcAngle(double distanceToWall, double wheelSpeed){
        double a = trajA;
        double b = trajB;

        double v = a * wheelSpeed - b;

        double tanget = Math.pow(v, 2) - Math.sqrt(
                Math.pow(v, 4) - 9.81 * (9.81 * distanceToWall * distanceToWall + 2 * wallHeigt * v * v)
        );

        tanget = tanget / 9.81 / distanceToWall;

        return clamp(Math.toDegrees(Math.atan(tanget)), 55, 77);

    }
    public double calcAngle(Pose pose1, Pose pose2, boolean convertToMeters,double wheelSpeed){
        return calcAngle(calculateDistance(pose1,pose2,convertToMeters),wheelSpeed);
    }

    public TrajectoryResult findLowestSafeTrajectory(double distanceToWall) {

        // Use a fine step for precision (0.1 degree)
        final double angleStep = 0.1;

        double minV0 = Double.MAX_VALUE;
        double optimalAngle = -1.0;

        // Iterate through all possible angles
        for (double angleDeg = minAngleDeg; angleDeg <= maxAngleDeg; angleDeg += angleStep) {

            // Convert angle to radians for Java's trigonometric functions
            double thetaRad = Math.toRadians(angleDeg);

            // Calculate intermediate trigonometric values
            double tanTheta = Math.tan(thetaRad);
            double cosTheta = Math.cos(thetaRad);

            // --- Formula Rearranged to solve for v0 ---
            // v0^2 = (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - (y - y0)))

            double denominatorTerm = (distanceToWall * tanTheta) - Y_DIFFERENCE;

            // CRITICAL CHECK: Denominator must be positive.
            // If it's zero or negative, the angle is too shallow (low) to reach the target height at that distance.
            if (denominatorTerm > 1e-9) {

                double v0Squared = (G_ACCELERATION * distanceToWall * distanceToWall) /
                        (2 * cosTheta * cosTheta * denominatorTerm);

                // Velocity must be real (positive square root)
                if (v0Squared > 0) {
                    double currentV0 = Math.sqrt(v0Squared);

                    // Check if this is the new minimum velocity found so far
                    if (currentV0 < minV0) {
                        minV0 = currentV0;
                        optimalAngle = angleDeg;
                    }
                }
            }
        }

        // Return the best result found
        if (optimalAngle != -1.0) {
            double requiredMotorRPM = minV0 * RPM_CONVERSION_FACTOR;
            return new TrajectoryResult(requiredMotorRPM, optimalAngle);
        } else {
            // no trajectory found given speed and angle
            return new TrajectoryResult(0.0, 0.0);
        }
    }

    public TrajectoryResult findLowestSafeTrajectory(Pose pose1, Pose pose2, boolean convertToMeters) {
        double distanceToWall = calculateDistance(pose1, pose2, convertToMeters);
        return findLowestSafeTrajectory(distanceToWall);
    }
}