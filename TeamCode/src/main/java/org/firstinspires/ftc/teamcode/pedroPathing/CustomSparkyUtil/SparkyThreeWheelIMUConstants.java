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

    /** The number of inches per tick of the encoder for forward movement
     * Default Value: .001989436789 */
    public double forwardTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for lateral movement (strafing)
     * Default Value: .001989436789 */
    public double strafeTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for turning
     * Default Value: .001989436789 */
    public double turnTicksToInches = .001989436789;

    /** The Y Offset of the Left Encoder (Deadwheel) from the center of the robot
     * Default Value: 1 */
    public double leftPodY = 1;

    /** The Y Offset of the Right Encoder (Deadwheel) from the center of the robot
     * Default Value: -1 */
    public double rightPodY = -1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot
     * Default Value: -2.5 */
    public double strafePodX = -2.5;

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * Default Value: "imu" */
    public String IMU_HardwareMapName = "imu";

    /** The name of the Left Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "leftFront" */
    public String leftEncoder_HardwareMapName = "leftFront";

    /** The name of the Right Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "rightRear" */
    public String rightEncoder_HardwareMapName = "rightRear";

    /** The name of the Strafe Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value:Default Value: "rightFront" */
    public String strafeEncoder_HardwareMapName = "rightFront";
    public String pinpoint_HardwareName = "pinpoint";

    /** The Orientation of the Control Hub (for IMU) on the Robot
     * Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the Left Encoder
     * Default Value: Encoder.REVERSE */
    public double leftEncoderDirection = Encoder.REVERSE;

    /** The direction of the Right Encoder
     * Default Value: Encoder.FORWARD */
    public double rightEncoderDirection = Encoder.REVERSE;

    /** The direction of the Strafe Encoder
     * Default Value: Encoder.FORWARD */
    public double strafeEncoderDirection = Encoder.FORWARD;

    /**
     * This is the IMU that will be used for localization.
     */
    //public CustomIMU imu = new RevHubIMU();

    /**
     * This creates a new ThreeWheelIMUConstants with default values.
     */
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

//    public SparkyThreeWheelIMUConstants customIMU(CustomIMU customIMU) {
//        this.imu = customIMU;
//        return this;
//    }

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
//        IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        leftEncoderDirection = Encoder.REVERSE;
        rightEncoderDirection = Encoder.REVERSE;
        strafeEncoderDirection = Encoder.FORWARD;
        pinpoint_HardwareName = "pinpoint";
//        imu = new RevHubIMU();
    }
}
