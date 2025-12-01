package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

/**
 * This is the PinpointConstants class. It holds many constants and parameters for the Pinpoint Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

@TargetApi(Build.VERSION_CODES.N)
public class SparkyPinpointConstants {

    /** The Y Offset of the Forward Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: 1 */
    public  double forwardPodY = 1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: -2.5 */
    public  double strafePodX = -2.5;

    /** The Unit of Distance that the Pinpoint uses to measure distance
     * Default Value: DistanceUnit.INCH */
    public  DistanceUnit distanceUnit = DistanceUnit.INCH;

    /** The name of the Pinpoint in the hardware map (name of the I2C port it is plugged into)
     * Default Value: "pinpoint" */
    public  String hardwareMapName = "pinpoint";

    /** Custom Yaw Scalar for the Pinpoint (overrides the calibration of the Pinpoint) */
    @SuppressLint("NewApi")
    public OptionalDouble yawScalar = OptionalDouble.empty();

    /** The Encoder Resolution for the Pinpoint. Used by default, but can be changed to a custom resolution.
     * Default Value: GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD */
    public  SparkyPinpointDriver.GoBildaOdometryPods encoderResolution = SparkyPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /** The Encoder Resolution for the Pinpoint. Unused by default, but can be used if you want to use a custom encoder resolution. */
    @SuppressLint("NewApi")
    public OptionalDouble customEncoderResolution = OptionalDouble.empty();

    /** The Encoder Direction for the Forward Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.REVERSED */
    public  SparkyPinpointDriver.EncoderDirection forwardEncoderDirection = SparkyPinpointDriver.EncoderDirection.REVERSED;

    /** The Encoder Direction for the Strafe Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.FORWARD */
    public  SparkyPinpointDriver.EncoderDirection strafeEncoderDirection = SparkyPinpointDriver.EncoderDirection.FORWARD;

    /**
     * This creates a new PinpointConstants with default values.
     */
    public SparkyPinpointConstants() {
        defaults();
    }

    public SparkyPinpointConstants forwardPodY(double forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public SparkyPinpointConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public SparkyPinpointConstants distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    public SparkyPinpointConstants hardwareMapName(String hardwareMapName) {
        this.hardwareMapName = hardwareMapName;
        return this;
    }

    public SparkyPinpointConstants yawScalar(double yawScalar) {
        this.yawScalar = OptionalDouble.of(yawScalar);
        return this;
    }

    public SparkyPinpointConstants encoderResolution(SparkyPinpointDriver.GoBildaOdometryPods encoderResolution) {
        this.encoderResolution = encoderResolution;
        return this;
    }

    public SparkyPinpointConstants customEncoderResolution(double customEncoderResolution) {
        this.customEncoderResolution = OptionalDouble.of(customEncoderResolution);
        return this;
    }

    public SparkyPinpointConstants forwardEncoderDirection(SparkyPinpointDriver.EncoderDirection forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public SparkyPinpointConstants strafeEncoderDirection(SparkyPinpointDriver.EncoderDirection strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public void defaults() {
        forwardPodY = 1;
        strafePodX = -2.5;
        distanceUnit = DistanceUnit.INCH;
        hardwareMapName = "pinpoint";
        yawScalar = OptionalDouble.empty();
        encoderResolution = SparkyPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        customEncoderResolution = OptionalDouble.empty();
        forwardEncoderDirection = SparkyPinpointDriver.EncoderDirection.REVERSED;
        strafeEncoderDirection = SparkyPinpointDriver.EncoderDirection.FORWARD;
    }
}
