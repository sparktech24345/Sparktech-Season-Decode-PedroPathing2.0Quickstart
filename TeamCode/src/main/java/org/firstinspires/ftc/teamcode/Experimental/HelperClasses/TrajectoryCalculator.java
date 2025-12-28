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
    final double RPM_CONVERSION_FACTOR = (60 / (2 * Math.PI * 0.036)) * (34.0 / 22.0) / 6;
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
}