package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is the Encoder class. This tracks the position of a motor of class DcMotorEx. The motor
 * must have an encoder attached. It can also get changes in position.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public class SparkyStrafeEncoder {
    private final GoBildaPinpointDriver pinpointDriver;
    private double previousPosition;
    private double currentPosition;
    private double multiplier;

    public final static double FORWARD = 1, REVERSE = -1;

    public SparkyStrafeEncoder(GoBildaPinpointDriver pinpoint) {
        pinpointDriver = pinpoint;
        multiplier = FORWARD;
        reset();
    }

    public void setDirection(double setMultiplier) {
        multiplier = setMultiplier;
    }

    public void reset() {
        previousPosition = pinpointDriver.getEncoderY();
        currentPosition = pinpointDriver.getEncoderY();
    }

    public void update() {
        previousPosition = currentPosition;
        currentPosition = pinpointDriver.getEncoderY();
    }
    public double getMultiplier() {
        return multiplier * 1;
    }
    public double getDeltaPosition() {
        return getMultiplier() * (currentPosition - previousPosition);
    }
}
