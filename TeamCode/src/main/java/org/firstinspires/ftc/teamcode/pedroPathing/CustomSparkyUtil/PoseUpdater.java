//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.*;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.ITDConstants;

public class PoseUpdater {
    private HardwareMap hardwareMap;
    private IMU imu;
    private Localizer localizer;
    private Pose startingPose;
    private Pose currentPose;
    private Pose previousPose;
    private Vector currentVelocity;
    private Vector previousVelocity;
    private Vector currentAcceleration;
    private Class<?> constants;
    private double xOffset;
    private double yOffset;
    private double headingOffset;
    private long previousPoseTime;
    private long currentPoseTime;

    public PoseUpdater(HardwareMap hardwareMap, Localizer localizer, Class<?> ITDConstants) {
        this.startingPose = new Pose(0.0, 0.0, 0.0);
        this.currentPose = this.startingPose;
        this.previousPose = this.startingPose;
        this.currentVelocity = new Vector();
        this.previousVelocity = new Vector();
        this.currentAcceleration = new Vector();
        this.xOffset = 0.0;
        this.yOffset = 0.0;
        this.headingOffset = 0.0;
        this.constants = ITDConstants;
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;
        if (localizer.getClass() != PinpointLocalizer.class) {
            try {
                localizer.resetIMU();
            } catch (InterruptedException var6) {
            }
        }
    }

    public PoseUpdater(HardwareMap hardwareMap, Class<?> ITDConstants) {
        this(hardwareMap, createLocalizer(hardwareMap),ITDConstants);
    }

    public PoseUpdater(HardwareMap hardwareMap, Localizer localizer) {
        this.startingPose = new Pose(0.0, 0.0, 0.0);
        this.currentPose = this.startingPose;
        this.previousPose = this.startingPose;
        this.currentVelocity = new Vector();
        this.previousVelocity = new Vector();
        this.currentAcceleration = new Vector();
        this.xOffset = 0.0;
        this.yOffset = 0.0;
        this.headingOffset = 0.0;
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;
        if (localizer.getClass() != PinpointLocalizer.class) {
            try {
                localizer.resetIMU();
            } catch (InterruptedException var4) {
            }
        }
    }

    public PoseUpdater(HardwareMap hardwareMap) {
        this(hardwareMap, createLocalizer(hardwareMap));
    }

    private static Localizer createLocalizer(HardwareMap hardwareMap) {
        /*switch (ITDConstants.ITDlocalizerConstants.) {
            case DRIVE_ENCODERS:
                return new DriveEncoderLocalizer(hardwareMap);
            case TWO_WHEEL:
                return new TwoWheelLocalizer(hardwareMap);
            case THREE_WHEEL:
                return new ThreeWheelLocalizer(hardwareMap);
            case THREE_WHEEL_IMU:
                return new ThreeWheelIMULocalizer(hardwareMap);
            case OTOS:
                return new OTOSLocalizer(hardwareMap);
            case PINPOINT:
                return new PinpointLocalizer(hardwareMap);
            default:
                throw new IllegalArgumentException("Unsupported localizer type");
        }

         */
        return new PinpointLocalizer(hardwareMap,ITDConstants.ITDlocalizerConstants);
    }

    public void update() {
        this.previousVelocity = this.getVelocity();
        this.previousPose = this.applyOffset(this.getRawPose());
        this.currentPose = null;
        this.currentVelocity = null;
        this.currentAcceleration = null;
        this.previousPoseTime = this.currentPoseTime;
        this.currentPoseTime = System.nanoTime();
        this.localizer.update();
    }

    public void setStartingPose(Pose set) {
        this.startingPose = set;
        this.previousPose = this.startingPose;
        this.previousPoseTime = System.nanoTime();
        this.currentPoseTime = System.nanoTime();
        this.localizer.setStartPose(set);
    }

    public void setCurrentPoseWithOffset(Pose set) {
        Pose currentPose = this.getRawPose();
        this.setXOffset(set.getX() - currentPose.getX());
        this.setYOffset(set.getY() - currentPose.getY());
        this.setHeadingOffset(SecondaryMathFunctions.getTurnDirection(currentPose.getHeading(), set.getHeading()) * SecondaryMathFunctions.getSmallestAngleDifference(currentPose.getHeading(), set.getHeading()));
    }

    public void setXOffset(double offset) {
        this.xOffset = offset;
    }

    public void setYOffset(double offset) {
        this.yOffset = offset;
    }

    public void setHeadingOffset(double offset) {
        this.headingOffset = offset;
    }

    public double getXOffset() {
        return this.xOffset;
    }

    public double getYOffset() {
        return this.yOffset;
    }

    public double getHeadingOffset() {
        return this.headingOffset;
    }

    public Pose applyOffset(Pose pose) {
        return new Pose(pose.getX() + this.xOffset, pose.getY() + this.yOffset, pose.getHeading() + this.headingOffset);
    }

    public void resetOffset() {
        this.setXOffset(0.0);
        this.setYOffset(0.0);
        this.setHeadingOffset(0.0);
    }

    public Pose getPose() {
        if (this.currentPose == null) {
            this.currentPose = this.localizer.getPose();
            return this.applyOffset(this.currentPose);
        } else {
            return this.applyOffset(this.currentPose);
        }
    }

    public Pose getRawPose() {
        if (this.currentPose == null) {
            this.currentPose = this.localizer.getPose();
            return this.currentPose;
        } else {
            return this.currentPose;
        }
    }

    public void setPose(Pose set) {
        this.resetOffset();
        this.localizer.setPose(set);
    }

    public Pose getPreviousPose() {
        return this.previousPose;
    }

    /*public Pose getDeltaPose() {
        Pose returnPose = this.getPose();
        returnPose.subtract(this.previousPose);
        return returnPose;
    }

     */

    public Vector getVelocity() {
        if (this.currentVelocity == null) {
            this.currentVelocity = this.localizer.getVelocityVector();
            return SecondaryMathFunctions.copyVector(this.currentVelocity);
        } else {
            return SecondaryMathFunctions.copyVector(this.currentVelocity);
        }
    }

    public double getAngularVelocity() {
        return SecondaryMathFunctions.getTurnDirection(this.previousPose.getHeading(), this.getPose().getHeading()) * SecondaryMathFunctions.getSmallestAngleDifference(this.getPose().getHeading(), this.previousPose.getHeading()) / ((double)(this.currentPoseTime - this.previousPoseTime) / Math.pow(10.0, 9.0));
    }

    public Vector getAcceleration() {
        if (this.currentAcceleration == null) {
            this.currentAcceleration = SecondaryMathFunctions.subtractVectors(this.getVelocity(), this.previousVelocity);
            this.currentAcceleration.setMagnitude(this.currentAcceleration.getMagnitude() / ((double)(this.currentPoseTime - this.previousPoseTime) / Math.pow(10.0, 9.0)));
            return SecondaryMathFunctions.copyVector(this.currentAcceleration);
        } else {
            return SecondaryMathFunctions.copyVector(this.currentAcceleration);
        }
    }

    public void resetHeadingToIMU() {
        if (this.imu != null) {
            this.localizer.setPose(new Pose(this.getPose().getX(), this.getPose().getY(), this.getNormalizedIMUHeading() + this.startingPose.getHeading()));
        }

    }

    public void resetHeadingToIMUWithOffsets() {
        if (this.imu != null) {
            this.setCurrentPoseWithOffset(new Pose(this.getPose().getX(), this.getPose().getY(), this.getNormalizedIMUHeading() + this.startingPose.getHeading()));
        }

    }

    public double getNormalizedIMUHeading() {
        return this.imu != null ? SecondaryMathFunctions.normalizeAngle(-this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) : 0.0;
    }

    public double getTotalHeading() {
        return this.localizer.getTotalHeading();
    }

    public Localizer getLocalizer() {
        return this.localizer;
    }

    public void resetIMU() throws InterruptedException {
        this.localizer.resetIMU();
    }
}
