package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;

import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.RevHubIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the ThreeWheelIMUConstants class. It holds many constants and parameters for the Three Wheel + IMU Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */


public class SparkyThreeWheelIMUConstants {

    public double forwardTicksToInches = .001989436789;

    public double strafeTicksToInches = .001989436789;
    public double turnTicksToInches = .001989436789;

    public double leftPodY = 1;

    public double rightPodY = -1;

    public double strafePodX = -2.5;

    public String IMU_HardwareMapName = "imu";

    public String leftEncoder_HardwareMapName = "leftFront";

    public String rightEncoder_HardwareMapName = "rightRear";

    public String strafeEncoder_HardwareMapName = "rightFront";
    public String pinpoint_HardwareName = "pinpoint";

    public RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    public double leftEncoderDirection = Encoder.REVERSE;

    public double rightEncoderDirection = Encoder.REVERSE;


    public double strafeEncoderDirection = Encoder.FORWARD;
    public SparkyThreeWheelIMUConstants() {
        defaults();
    }

    public SparkyThreeWheelIMUConstants forwardTicksToInches(double forwardTicksToInches) {
        this.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public SparkyThreeWheelIMUConstants strafeTicksToInches(double strafeTicksToInches) {
        this.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public SparkyThreeWheelIMUConstants turnTicksToInches(double turnTicksToInches) {
        this.turnTicksToInches = turnTicksToInches;
        return this;
    }

    public SparkyThreeWheelIMUConstants leftPodY(double leftPodY) {
        this.leftPodY = leftPodY;
        return this;
    }

    public SparkyThreeWheelIMUConstants rightPodY(double rightPodY) {
        this.rightPodY = rightPodY;
        return this;
    }

    public SparkyThreeWheelIMUConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public SparkyThreeWheelIMUConstants setPinpoint_HardwareName(String pinpoint_HardwareName) {
        this.pinpoint_HardwareName = pinpoint_HardwareName;
        return this;
    }

    public SparkyThreeWheelIMUConstants leftEncoder_HardwareMapName(String leftEncoder_HardwareMapName) {
        this.leftEncoder_HardwareMapName = leftEncoder_HardwareMapName;
        return this;
    }

    public SparkyThreeWheelIMUConstants rightEncoder_HardwareMapName(String rightEncoder_HardwareMapName) {
        this.rightEncoder_HardwareMapName = rightEncoder_HardwareMapName;
        return this;
    }

    public SparkyThreeWheelIMUConstants strafeEncoder_HardwareMapName(String strafeEncoder_HardwareMapName) {
        this.strafeEncoder_HardwareMapName = strafeEncoder_HardwareMapName;
        return this;
    }

    public SparkyThreeWheelIMUConstants IMU_Orientation(RevHubOrientationOnRobot IMU_Orientation) {
        this.IMU_Orientation = IMU_Orientation;
        return this;
    }

    public SparkyThreeWheelIMUConstants leftEncoderDirection(double leftEncoderDirection) {
        this.leftEncoderDirection = leftEncoderDirection;
        return this;
    }

    public SparkyThreeWheelIMUConstants rightEncoderDirection(double rightEncoderDirection) {
        this.rightEncoderDirection = rightEncoderDirection;
        return this;
    }

    public SparkyThreeWheelIMUConstants strafeEncoderDirection(double strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }


    public void defaults() {
        forwardTicksToInches = .001989436789;
        strafeTicksToInches = .001989436789;
        turnTicksToInches = .001989436789;
        leftPodY = 1;
        rightPodY = -1;
        strafePodX = -2.5;
        IMU_HardwareMapName = "imu";
        leftEncoder_HardwareMapName = "leftFront";
        rightEncoder_HardwareMapName = "rightRear";
        strafeEncoder_HardwareMapName = "rightFront";
        leftEncoderDirection = Encoder.REVERSE;
        rightEncoderDirection = Encoder.REVERSE;
        strafeEncoderDirection = Encoder.FORWARD;
        pinpoint_HardwareName = "pinpoint";
    }
}
