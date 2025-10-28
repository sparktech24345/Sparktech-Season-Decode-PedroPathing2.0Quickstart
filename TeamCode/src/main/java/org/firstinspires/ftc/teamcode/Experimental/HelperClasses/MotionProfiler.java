package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

//a motion profiler snatched off the web and slightly modified
public class MotionProfiler {
    private double maxVel;
    private double maxAccel;
    private double profiledPos;
    private double profiledVel;

    private double syncThreshold;

    public MotionProfiler(double maxVel, double maxAccel, double syncThreshold) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.syncThreshold = syncThreshold;
        this.profiledPos = 0;
        this.profiledVel = 0;
    }

    public void reset(double currentPos) {
        profiledPos = currentPos;
        profiledVel = 0;
    }

    public double update(double currentPos, double targetPos, double dt) {
        // --- Sync ---
        double diff = currentPos - profiledPos;
        if (Math.abs(diff) > syncThreshold) {
            //been pushed, re-sync to real position
            profiledPos = currentPos;
            profiledVel = 0;
        }

        double error = targetPos - profiledPos;
        double dir = Math.signum(error);

        double desiredVel = Math.sqrt(2 * maxAccel * Math.abs(error));
        desiredVel = Math.min(desiredVel, maxVel);
        double desiredVelSigned = desiredVel * dir;

        double velDiff = desiredVelSigned - profiledVel;
        double maxVelChange = maxAccel * dt;
        if (Math.abs(velDiff) > maxVelChange) {
            profiledVel += Math.signum(velDiff) * maxVelChange;
        } else {
            profiledVel = desiredVelSigned;
        }
        profiledPos += profiledVel * dt;
        if ((targetPos - profiledPos) * dir < 0) {
            profiledPos = targetPos;
            profiledVel = 0;
        }

        return profiledPos;
    }
}
