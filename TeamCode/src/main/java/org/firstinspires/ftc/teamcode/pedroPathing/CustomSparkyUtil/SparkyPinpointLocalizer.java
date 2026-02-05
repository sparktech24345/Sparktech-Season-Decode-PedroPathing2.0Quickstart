package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;

import android.annotation.SuppressLint;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;

import java.util.Objects;

/**
 * This is the Pinpoint class. This class extends the Localizer superclass and is a
 * localizer that uses the two wheel odometry set up with the IMU to have more accurate heading
 * readings. The diagram below, which is modified from Road Runner, shows a typical set up.
 *
 * @author Logan Nash
 * @author Havish Sripada 12808 - RevAmped Robotics
 * @author Ethan Doak - GoBilda
 * @version 2.0, 6/30/2025
 */
public class SparkyPinpointLocalizer implements Localizer {
    private final SparkyPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private Pose currentVelocity;
    private Pose pinpointPose;


    public SparkyPinpointLocalizer(HardwareMap map, SparkyPinpointConstants constants){ this(map, constants, new Pose());}


    @SuppressLint("NewApi")
    public SparkyPinpointLocalizer(HardwareMap map, SparkyPinpointConstants constants, Pose setStartPose){

        odo = map.get(SparkyPinpointDriver.class,constants.hardwareMapName);
        setOffsets(constants.forwardPodY, constants.strafePodX, constants.distanceUnit);

        if(constants.yawScalar.isPresent()) {
            odo.setYawScalar(constants.yawScalar.getAsDouble());
        }

        if(constants.customEncoderResolution.isPresent()) {
            odo.setEncoderResolution(constants.customEncoderResolution.getAsDouble(), constants.distanceUnit);
        } else {
            odo.setEncoderResolution(constants.encoderResolution);
        }

        odo.setEncoderDirections(constants.forwardEncoderDirection, constants.strafeEncoderDirection);

        setStartPose(setStartPose);
        totalHeading = 0;
        pinpointPose = startPose;
        currentVelocity = new Pose();
        previousHeading = setStartPose.getHeading();
    }


    @Override
    public Pose getPose() {
        return pinpointPose;
    }


    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = pinpointPose.rotate(-startPose.getHeading(), false).minus(startPose);
            setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }

        this.startPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        odo.setPosition(PoseConverter.poseToPose2D(setPose, PedroCoordinates.INSTANCE));
        pinpointPose = setPose;
        previousHeading = setPose.getHeading();
    }

    @Override
    public void update() {
        odo.update();
        Pose currentPinpointPose = PoseConverter.pose2DToPose(odo.getPosition(), PedroCoordinates.INSTANCE);
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentPinpointPose.getHeading());
        previousHeading = currentPinpointPose.getHeading();
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized()));
        pinpointPose = currentPinpointPose;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return odo.getEncoderY();
    }

    @Override
    public double getLateralMultiplier() {
        return odo.getEncoderX();
    }

    @Override
    public double getTurningMultiplier() {
        return odo.getYawScalar();
    }

    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        odo.setOffsets(xOffset, yOffset, unit);
    }

    @Override
    public void resetIMU() {
        resetPinpoint();
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void recalibrate() {
        odo.recalibrateIMU();
    }

    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    public SparkyPinpointDriver getPinpoint() {
        return odo;
    }
}
