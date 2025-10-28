package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

//a motion profiler snatched off the web and slightly modified
public class MotionProfiler {
    private double maxVel;
    private double maxAccel;
    private double profiledPos;
    private double profiledVel;

    public MotionProfiler(double maxVel, double maxAccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.profiledPos = 0;
        this.profiledVel = 0;
    }

    public void reset(double currentPos) {
        profiledPos = currentPos;
        profiledVel = 0;
    }


    public double update(double currentPos, double targetPos, double dt) {
        double error = targetPos - profiledPos;
        double dir = Math.signum(error);


        double desiredVel = Math.sqrt(2 * maxAccel * Math.abs(error));
        desiredVel = Math.min(desiredVel, maxVel);

        double desiredVelSigned = desiredVel * dir;


        double velChange = desiredVelSigned - profiledVel;
        double maxVelChange = maxAccel * dt;
        if (Math.abs(velChange) > maxVelChange) {
            profiledVel += Math.signum(velChange) * maxVelChange;
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
