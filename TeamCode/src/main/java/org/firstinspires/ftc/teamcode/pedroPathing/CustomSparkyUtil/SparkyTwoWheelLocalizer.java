package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;

import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;

/**
 * This is the TwoWheelLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the two wheel odometry with IMU set up.
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */

public class SparkyTwoWheelLocalizer implements Localizer {
    private PinpointLocalizer imu;
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private final NanoTimer timer;
    private long deltaTimeNano;
    private final Encoder forwardEncoder;
    private final Encoder strafeEncoder;
    private final double strafePodX;
    private final double forwardPodY;
    private double previousIMUOrientation;
    private double deltaRadians;
    private double totalHeading;
    public static double FORWARD_TICKS_TO_INCHES;
    public static double STRAFE_TICKS_TO_INCHES;
 public SparkyTwoWheelLocalizer(HardwareMap map, TwoWheelConstants constants) {
        this(map, constants, new Pose());
    }

    public SparkyTwoWheelLocalizer(HardwareMap map, TwoWheelConstants constants,Pose setStartPose) {
        FORWARD_TICKS_TO_INCHES = constants.forwardTicksToInches;
        STRAFE_TICKS_TO_INCHES = constants.strafeTicksToInches;
        imu = new PinpointLocalizer(map, ConstantsDecode.pinpointConstants);
        strafePodX = constants.strafePodX;
        forwardPodY = constants.forwardPodY;

        forwardEncoder = new Encoder(map.get(DcMotorEx.class, "intakemotor"));
        strafeEncoder = new Encoder(map.get(DcMotorEx.class, "outakerightmotor"));

        forwardEncoder.setDirection(constants.forwardEncoderDirection);
        strafeEncoder.setDirection(constants.strafeEncoderDirection);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();

        previousIMUOrientation = MathFunctions.normalizeAngle(imu.getPose().getHeading());
        deltaRadians = 0;
    }
    @Override
    public Pose getPose() {
        return startPose.plus(displacementPose);
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
        startPose = setStart;
    }

    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3,3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }
@Override
    public void setPose(Pose setPose) {
        displacementPose = setPose.minus(startPose);
        resetEncoders();
    }
    @Override
    public void update() {
        imu.update();
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose = displacementPose.plus(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano / Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);
    }
    public void updateEncoders() {
        forwardEncoder.update();
        strafeEncoder.update();

        double currentIMUOrientation = MathFunctions.normalizeAngle(imu.getPose().getHeading());
        deltaRadians = MathFunctions.getTurnDirection(previousIMUOrientation, currentIMUOrientation) * MathFunctions.getSmallestAngleDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;
    }
public void resetEncoders() {
        forwardEncoder.reset();
        strafeEncoder.reset();
    }
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * forwardEncoder.getDeltaPosition() - forwardPodY * deltaRadians);
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * strafeEncoder.getDeltaPosition() - strafePodX * deltaRadians);
        returnMatrix.set(2,0, deltaRadians);
        return returnMatrix;
    }
    public double getTotalHeading() {
        return totalHeading;
    }
    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }
    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }
    public double getTurningMultiplier() {
        return 1;
    }
    public void resetIMU() {
        imu.resetIMU();
    }
    @Override
    public double getIMUHeading() {
        return imu.getPose().getHeading();
    }
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }
}