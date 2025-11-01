package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

/**
 * MotionProfiling.java
 * Provides a single, simple static utility method for calculating the theoretical
 * position along a trapezoidal motion profile at a given elapsed time 't'.
 * This version is optimized for position-only control loops.
 * All units (position, velocity, acceleration) should be consistent (e.g., inches/sec).
 */
public class MotionProfiling {

    /**
     * Calculates the smoothed theoretical position along a trapezoidal profile
     * in a single pass.
     * * @param targetPosition The final desired position (units, can be negative).
     * @param maxVelocity The maximum allowed velocity (units/sec, always positive).
     * @param maxAcceleration The maximum allowed acceleration (units/sec^2, always positive).
     * @param t The elapsed time since the start of the motion (seconds).
     * @return The theoretical position at time t (units).
     */
    public static double getTrapezoidPosition(double targetPosition, double maxVelocity, double maxAcceleration, double t) {

        // --- 1. Determine Motion Direction and Absolute Distance ---
        final double sign = Math.signum(targetPosition);
        final double absTarget = Math.abs(targetPosition);
        final double V_max = Math.abs(maxVelocity);
        final double A_max = Math.abs(maxAcceleration);
        final double totalDistance = absTarget;

        // --- 2. Pre-calculate Profile Timing and Distances ---
        double t_accel;    // Time spent accelerating
        double d_accel;    // Distance covered during acceleration
        double t_cruise;   // Time spent at max velocity
        double t_total;    // Total time for the profile

        // Distance needed to accelerate to V_max and decelerate from V_max to 0
        final double minDistanceToReachMaxVel = (V_max * V_max) / (2.0 * A_max);

        if (totalDistance <= (2.0 * minDistanceToReachMaxVel)) {
            // --- Triangular Profile (Never reaches V_max) ---
            double V_peak = Math.sqrt(A_max * totalDistance);
            t_accel = V_peak / A_max;
            d_accel = totalDistance / 2.0; // Accel distance is half total distance
            t_cruise = 0.0;
            t_total = 2.0 * t_accel;
        } else {
            // --- Trapezoidal Profile (Reaches V_max) ---
            t_accel = V_max / A_max;
            d_accel = 0.5 * A_max * t_accel * t_accel;
            double cruiseDist = totalDistance - (2.0 * d_accel);
            t_cruise = cruiseDist / V_max;
            t_total = (2.0 * t_accel) + t_cruise;
        }

        // --- 3. Clamp Time and Initialize Position ---

        // Ensure time t is within the bounds of the motion profile
        t = Math.max(0.0, Math.min(t, t_total));
        double absPosition = 0.0;

        // --- 4. Calculate Position based on Phase ---

        if (t <= t_accel) {
            // Phase 1: Acceleration
            // P(t) = 0.5 * A_max * t^2
            absPosition = 0.5 * A_max * t * t;

        } else if (t <= (t_accel + t_cruise)) {
            // Phase 2: Cruise
            // P(t) = d_accel + V_max * (t - t_accel)
            absPosition = d_accel + V_max * (t - t_accel);

        } else {
            // Phase 3: Deceleration
            double t_remaining = t_total - t;
            // P(t) = P_total - 0.5 * A_max * (t_total - t)^2
            absPosition = totalDistance - (0.5 * A_max * t_remaining * t_remaining);
        }

        // --- 5. Apply Sign and Return ---
        return absPosition * sign;
    }
}
