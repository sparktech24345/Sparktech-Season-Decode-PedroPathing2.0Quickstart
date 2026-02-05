package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;
//
//import com.pedropathing.ftc.localization.CustomIMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyThreeWheelIMUConstants;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import com.pedropathing.ftc.localization.Encoder;
//import com.pedropathing.localization.Localizer;
//import com.pedropathing.math.Matrix;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.math.MathFunctions;
//import com.pedropathing.math.Vector;
//import com.pedropathing.util.NanoTimer;
//
///**
// * This is the ThreeWheelIMULocalizer class. This class extends the Localizer superclass and is a
// * localizer that uses the three wheel odometry set up with the IMU to have more accurate heading
// * readings.
// *
// * @author Logan Nash
// * @author Anyi Lin - 10158 Scott's Bots
// * @version 1.0, 7/9/2024
// */
//
//public class SparkyThreeWheelIMULocalizer implements Localizer {
//    private Pose startPose;
//    private Pose displacementPose;
//    private Pose currentVelocity;
//    private Matrix prevRotationMatrix;
//    private final NanoTimer timer;
//    private long deltaTimeNano;
//    private Encoder rightEncoder;
//    private SparkyLeftEncoder leftEncoder;
//    private SparkyStrafeEncoder strafeEncoder;
//    private final Pose leftEncoderPose;
//    private final Pose rightEncoderPose;
//    private final Pose strafeEncoderPose;
//    public GoBildaPinpointDriver pinpoint;
//    private double previousIMUOrientation;
//    private double deltaRadians;
//    private double totalHeading;
//    public static double FORWARD_TICKS_TO_INCHES;
//    public static double STRAFE_TICKS_TO_INCHES;
//    public static double TURN_TICKS_TO_RADIANS;
//
//    public static boolean useIMU = true;
//
//    public SparkyThreeWheelIMULocalizer(HardwareMap map, SparkyThreeWheelIMUConstants constants) {
//        this(map, constants, new Pose());
//    }
//
//    public SparkyThreeWheelIMULocalizer(HardwareMap map, SparkyThreeWheelIMUConstants constants, Pose setStartPose) {
//        FORWARD_TICKS_TO_INCHES = constants.forwardTicksToInches;
//        STRAFE_TICKS_TO_INCHES = constants.strafeTicksToInches;
//        TURN_TICKS_TO_RADIANS = constants.turnTicksToInches;
//
//        pinpoint = map.get(GoBildaPinpointDriver.class, constants.pinpoint_HardwareName);
//
//        leftEncoderPose = new Pose(0, constants.leftPodY, 0);
//        rightEncoderPose = new Pose(0, constants.rightPodY, 0);
//        strafeEncoderPose = new Pose(constants.strafePodX, 0, Math.toRadians(90));
//
//        pinpoint.initialize();
//
//
//        leftEncoder = new SparkyLeftEncoder(pinpoint);
//        rightEncoder = new Encoder(map.get(DcMotorEx.class, constants.rightEncoder_HardwareMapName));
//        strafeEncoder = new SparkyStrafeEncoder(pinpoint);
//
//
//        leftEncoder.setDirection(constants.leftEncoderDirection);
//        rightEncoder.setDirection(constants.rightEncoderDirection);
//        strafeEncoder.setDirection(constants.strafeEncoderDirection);
//
//        setStartPose(setStartPose);
//        timer = new NanoTimer();
//        deltaTimeNano = 1;
//        displacementPose = new Pose();
//        currentVelocity = new Pose();
//        totalHeading = 0;
//
//        resetEncoders();
//    }
//
//    @Override
//    public Pose getPose() {
//        return startPose.plus(displacementPose);
//    }
//
//    @Override
//    public Pose getVelocity() {
//        return currentVelocity;
//    }
//
//    @Override
//    public Vector getVelocityVector() {
//        return currentVelocity.getAsVector();
//    }
//
//    @Override
//    public void setStartPose(Pose setStart) {
//        startPose = setStart;
//    }
//
//    public void setPrevRotationMatrix(double heading) {
//        prevRotationMatrix = new Matrix(3,3);
//        prevRotationMatrix.set(0, 0, Math.cos(heading));
//        prevRotationMatrix.set(0, 1, -Math.sin(heading));
//        prevRotationMatrix.set(1, 0, Math.sin(heading));
//        prevRotationMatrix.set(1, 1, Math.cos(heading));
//        prevRotationMatrix.set(2, 2, 1.0);
//    }
//
//    @Override
//    public void setPose(Pose setPose) {
//        displacementPose = setPose.minus(startPose);
//        resetEncoders();
//    }
//
//    @Override
//    public void update() {
//        deltaTimeNano = timer.getElapsedTime();
//        timer.resetTimer();
//
//        updateEncoders();
//        Matrix robotDeltas = getRobotDeltas();
//        Matrix globalDeltas;
//        setPrevRotationMatrix(getPose().getHeading());
//
//        Matrix transformation = new Matrix(3,3);
//        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
//            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
//            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
//            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
//            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
//            transformation.set(2, 2, 1.0);
//        } else {
//            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
//            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
//            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
//            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
//            transformation.set(2, 2, 1.0);
//        }
//
//        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);
//
//        displacementPose = displacementPose.plus(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
//        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano / Math.pow(10.0, 9)));
//
//        totalHeading += globalDeltas.get(2, 0);
//    }
//
//    public void updateEncoders() {
//        leftEncoder.update();
//        rightEncoder.update();
//        strafeEncoder.update();
//
//        double currentIMUOrientation = MathFunctions.normalizeAngle(pinpoint.getHeading(AngleUnit.RADIANS));
//        deltaRadians = MathFunctions.getTurnDirection(previousIMUOrientation, currentIMUOrientation) * MathFunctions.getSmallestAngleDifference(currentIMUOrientation, previousIMUOrientation);
//        previousIMUOrientation = currentIMUOrientation;
//    }
//
//    public void resetEncoders() {
//        leftEncoder.reset();
//        rightEncoder.reset();
//        strafeEncoder.reset();
//    }
//
//    public Matrix getRobotDeltas() {
//        Matrix returnMatrix = new Matrix(3,1);
//        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * (rightEncoder.getDeltaPosition() * leftEncoderPose.getY() - leftEncoder.getDeltaPosition() * rightEncoderPose.getY()) / (leftEncoderPose.getY() - rightEncoderPose.getY()));
//        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition() - strafeEncoderPose.getX() * ((rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY()))));
//        if (MathFunctions.getSmallestAngleDifference(0, deltaRadians) > 0.00005 && useIMU) {
//            returnMatrix.set(2, 0, deltaRadians);
//        } else {
//            returnMatrix.set(2,0, TURN_TICKS_TO_RADIANS * (rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY()));
//        }
//        return returnMatrix;
//    }
//
//    public double getTotalHeading() {
//        return totalHeading;
//    }
//
//    public double getForwardMultiplier() {
//        return FORWARD_TICKS_TO_INCHES;
//    }
//
//    public double getLateralMultiplier() {
//        return STRAFE_TICKS_TO_INCHES;
//    }
//    public double getTurningMultiplier() {
//        return TURN_TICKS_TO_RADIANS;
//    }
//    public void resetIMU() {
//        pinpoint.recalibrateIMU();
//        pinpoint.resetPosAndIMU();
//    }
//    @Override
//    public double getIMUHeading() {
//        return pinpoint.getHeading(AngleUnit.RADIANS);
//    }
//    public boolean isNAN() {
//        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
//    }
//}
