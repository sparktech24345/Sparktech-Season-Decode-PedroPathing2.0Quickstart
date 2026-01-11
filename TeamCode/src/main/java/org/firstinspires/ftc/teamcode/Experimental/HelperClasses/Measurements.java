package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import android.util.Pair;

public enum Measurements {
    P1(1.087, 860, 270),
    P2(1.474, 900, 270),
    P3(1.805, 990, 270),
    P4(1.917, 1000, 267),
    P5(2.618, 1150, 263);

    public double distance;
    public double velocity;
    public double angle;

    Measurements(double distance, double velocity, double angle) {
        this.distance = distance;
        this.velocity = velocity;
        this.angle = angle;
    }

    public double angle() { return angle; }
    public double velocity() { return velocity; }
    public double distance() { return distance; }

    public static Pair<Measurements, Measurements> getClosestFromDistance(double distance) {
        if (distance >= P1.distance && distance <= P2.distance) {
            return new Pair<>(P1, P2);
        } else if (distance >= P2.distance && distance <= P3.distance) {
            return new Pair<>(P2, P3);
        } else if (distance >= P3.distance && distance <= P4.distance) {
            return new Pair<>(P3, P4);
        } else if (distance >= P4.distance && distance <= P5.distance) {
            return new Pair<>(P4, P5);
        } else return new Pair<>(P1, P1);
    }
}
