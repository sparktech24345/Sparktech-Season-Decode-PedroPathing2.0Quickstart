package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Components.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.*;

import android.graphics.Color;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Components;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.ActionSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.HoldAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BezierCurveTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

import java.util.List;

@com.acmerobotics.dashboard.config.Config
@Autonomous(name = "Small Triangle Auto BLUE", group = "AAA")
public class SmallTriangleNew extends ComplexOpMode {
    //private AutoRecorder recorder;
    public Limelight3A limelight3A;
    public static Config cfg;
    private boolean shouldFire;
    public static boolean isMoving;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime resetLeftBallColorTimer = new ElapsedTime();
    public static boolean doIntakePulse = false;
    public static boolean shouldHoldTurretForCameraScan = false;
    public static boolean shouldCheckColorSensors = false;
    public static boolean doOnce = false;

    protected NormalizedColorSensor colorSensorRight;
    protected NormalizedColorSensor colorSensorLeft;
    protected NormalizedRGBA rightSensorColors;
    protected NormalizedRGBA leftSensorColors;
    final float[] hsvRightSensorColors = new float[3];
    final float[] hsvLeftSensorColors = new float[3];
    protected BallColorSet_Decode actualRightSensorDetectedBall;
    protected BallColorSet_Decode calculatedRightSensorDetectedBall;
    protected BallColorSet_Decode actualLeftSensorDetectedBall;
    protected BallColorSet_Decode calculatedLeftSensorDetectedBall;
    protected BallColorSet_Decode ballToFire;
    public static boolean hasBallInRightChamber = false;
    public static boolean hasBallInLeftChamber = false;
    public static boolean shouldRemoveBalls = false;
    private boolean shouldResetRightSensorBall = false;
    public static boolean shouldPullFromQueue = false;
    public static boolean shouldSwitchChannel = false;
    BallColorQueue ballColorQueue = new BallColorQueue();
    public static double velocity = 1480;
    public static double angle = 266;
    public static double rotationNeededForCameraScan = 13; // about 13 degrees to scan with limelight
    public static int camId = 23;
    public static int wentTooNumber2 = 23;
    private Pose starter = pose( 0, 12.85, 90); // would also be around 1.4x
    private Pose small_triangle_shoot = pose(1.5, 8, 90);
    private Pose parkPose = pose(1, 22, 90);
    private Pose fininshHPCollectPose = pose(0.5,47,90); // hp collect
    private Pose secondZoneCameraCollect = pose(17.8 + 0.5, 44, 90); /// CHECK THIS slightly more up spot
    private Pose thirdZoneCameraCollect = pose(33.96, 44, 90);
    private Pose thirdRowCollectDone = pose(30, 37, 90); // third row done
    private Pose secondRowCollectDone = pose(51.7, 37.5, 90);
    private Pose firstRowCollectDone = pose(78.1, 35.8, 90);
    private Pose gateCollect = pose(53.8, 40.2, 70);
    private Pose tipBigTriangleShooting = pose(67, -5.3, 180);
    private Pose middleBigTriangleShooting = pose(88.2, -4.7, 180);
    private Pose middleBigTriangleShootingTurned90Deg = pose(88.2, -4.7, 90);
    private Pose parkedBigTriangleShooting = pose(104.2, 0, 180);
    private Pose autoCloseStart = pose(123.1, 30.4, -129);
    private Pose gateOpen = pose(60.1, 36.7, 90);
    private Pose bezierHelper1 = pose(32, 4, 90);

    @Override
    public void telemetry() {
        publicTelemetry.addData("robot rotation", Math.toDegrees(ComplexFollower.getCurrentPose().getHeading()));
        publicTelemetry.addData("robot Y", ComplexFollower.getCurrentPose().getY());
        publicTelemetry.addData("robot X", ComplexFollower.getCurrentPose().getX());
        publicTelemetry.addData("current velocity", TurretSpinMotor.getVelocity());
    }

    @Override
    public void update() {
        // recorder.update();
        isMoving = ComplexFollower.instance().isBusy();
        if (shouldCheckColorSensors) HandleColors();
        if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(fininshHPCollectPose) && !ComplexFollower.done()) ComplexFollower.interrupt();
        firingTurret(shouldFire);
        pulseIntake(doIntakePulse);
        if (timer.milliseconds() > 29000 + 800) {
            EmergencyOverrideAtTheEnd();
            timer.reset();
        }
    }

    @Override
    public void initialize() {
        makeConfig();
        Components.init();
        //recorder = new AutoRecorder();
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        convertPoses();
        shouldFire = false; lastGateState = 0; resetLeftBallColorTimer.reset();
        shouldRemoveBalls = false;shouldResetRightSensorBall = false; doIntakePulse = false;
        wentTooNumber2 = 0;

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        doOnce = true;
    }

    @Override
    public void init_update() {
        if (doOnce) {
            ComplexFollower.unInit();
            ComplexFollower.init(ConstantsDecode::createFollowerDecodeFarAuto);
        }
        doOnce = false;
        TurretRotateMotor.setState(cfg.rotationForInitSmallTriangle);
        useCamera();
        publicTelemetry.addData("id", camId);
    }

    @Override
    public void on_start() {
        ComplexFollower.setPose(starter);
        timer.reset();
        makeAuto();
    }

    @Override
    public void on_stop() {
        //recorder.save();
        passPose();
    }
    private void makeAuto() {
        publicQueuer.addToQueue(
                // preload
                new GeneralAction(() -> shouldFire = true), // prep outtake
                new StateAction(IntakeMotor.states.FULL),
                new DelayAction(1100), // revving up outtake
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                // end of preload

                //first hp collect with the preset balls there
                new MoveAction(fininshHPCollectPose),
                new DelayAction(600),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(200),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                // end of hp collect and shooting


                // third row collecting and shooting
                new MoveAction(thirdRowCollectDone, BezierCurveTypes.ConstantHeading,thirdRowCollectDone.getHeading(), bezierHelper1),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(200),
                new GeneralAction(fireUnsortedBalls),
                new GeneralAction(() -> limelight3A.pipelineSwitch(1)), // switch channel only once
                new DelayAction(1200),
                // finished third row shooting


                // 4th cycle, camera collecting
                new GeneralAction(() -> shouldSwitchChannel = true),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = true),
                new DelayAction(700),
                new GeneralAction(scanForBallsAndPlanPath),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = false),
                // actual collect and firing
                new MoveAction(true),
                new DelayAction(400),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                // finished firing camera cycle



                // 5th cycle, camera collecting
                new GeneralAction(() -> shouldSwitchChannel = true),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = true),
                new DelayAction(700),
                new GeneralAction(scanForBallsAndPlanPath),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = false),
                // actual collect and firing
                new MoveAction(true),
                new DelayAction(400),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                // finished firing camera cycle




                // 6th cycle, camera collecting
                new GeneralAction(() -> shouldSwitchChannel = true),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = true),
                new DelayAction(700),
                new GeneralAction(scanForBallsAndPlanPath),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = false),
                // actual collect and firing
                new MoveAction(true),
                new DelayAction(400),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1300),
                // finished firing camera cycle



                new StateAction(IntakeMotor.states.OFF),
                new GeneralAction(turnStuffOff),
                new MoveAction(parkPose)
        );
    }
    public Runnable scanForBallsAndPlanPath = () -> {
        int cameraCase = (int) getBallNumber();
        //if (wentTooNumber2 >0) cameraCase = 1; not needed with new rolers
        switch (cameraCase) {
            case 1: GlobalStorage.futureMoveActionTargetPose = fininshHPCollectPose; break;
            case 2: GlobalStorage.futureMoveActionTargetPose = secondZoneCameraCollect; wentTooNumber2++; break;
            //case 3: GlobalStorage.futureMoveActionTargetPose = thirdZoneCameraCollect; break;
            default: GlobalStorage.futureMoveActionTargetPose = fininshHPCollectPose; break;
        }
    };
    Runnable checkEmptyIntake = () -> {
        if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.NoBall || calculatedRightSensorDetectedBall == BallColorSet_Decode.NoBall) {
            // go to collect again
            publicQueuer.executeNow(
                    new GeneralAction(scanForBallsAndPlanPath),
                    new MoveAction(true)
            );
        }

    };
    public boolean checkIfNoBall() {
        return calculatedLeftSensorDetectedBall == BallColorSet_Decode.NoBall || calculatedRightSensorDetectedBall == BallColorSet_Decode.NoBall;
    }

    Runnable prepQueueToFireSortedBall = () -> {
        ballColorQueue.clearQueue();
        switch (camId) {
            case 23:
                ballColorQueue.add(BallColorSet_Decode.Purple);
                ballColorQueue.add(BallColorSet_Decode.Purple);
                ballColorQueue.add(BallColorSet_Decode.Green);
                break;

            case 22:
                ballColorQueue.add(BallColorSet_Decode.Purple);
                ballColorQueue.add(BallColorSet_Decode.Green);
                ballColorQueue.add(BallColorSet_Decode.Purple);
                break;

            case 21:
                ballColorQueue.add(BallColorSet_Decode.Green);
                ballColorQueue.add(BallColorSet_Decode.Purple);
                ballColorQueue.add(BallColorSet_Decode.Purple);
                break;
        }
    };
    Runnable fireSortedBall = () -> {
        ballToFire = ballColorQueue.pull();

        if (ballToFire == calculatedRightSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
            publicQueuer.executeNow(new ActionSequence(
                    new StateAction(RightGateServo.states.OPEN),
                    new DelayAction(300),
                    new GeneralAction(() -> calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall),
                    new StateAction(RightGateServo.states.CLOSED)
            ));
        }
        else if (ballToFire == calculatedLeftSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
            publicQueuer.executeNow(new ActionSequence(
                    new StateAction(LeftGateServo.states.OPEN),
                    new DelayAction(300),
                    new GeneralAction(() -> calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall),
                    new StateAction(LeftGateServo.states.CLOSED)
            ));
        }
        else
        {
            if (calculatedLeftSensorDetectedBall != BallColorSet_Decode.NoBall) {
                publicQueuer.executeNow(new ActionSequence(
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(300),
                        new GeneralAction(() -> calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
            }
            else if (calculatedRightSensorDetectedBall != BallColorSet_Decode.NoBall) {
                publicQueuer.executeNow(new ActionSequence(
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(300),
                        new GeneralAction(() -> calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new StateAction(RightGateServo.states.CLOSED)
                ));
            }
            else {
                publicQueuer.executeNow(new ActionSequence(
                        new StateAction(RightGateServo.states.OPEN),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(500),
                        new GeneralAction(() -> calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new GeneralAction(() -> calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
            }

        }
        ballToFire = BallColorSet_Decode.NoBall;
    };
    Runnable fireUnsortedBalls = () -> publicQueuer.executeNow(new ActionSequence(
            new StateAction(RightGateServo.states.OPEN),
            new DelayAction(timerToCloseGate),
            new StateAction(RightGateServo.states.CLOSED),
            new DelayAction(timer3),
            new StateAction(LeftGateServo.states.OPEN),
            new DelayAction(timer4),
            new StateAction(RightGateServo.states.OPEN),
            new DelayAction(1000),
            new StateAction(RightGateServo.states.CLOSED),
            new StateAction(LeftGateServo.states.CLOSED)
    ));
    Runnable turnStuffOff = () -> {
        shouldFire = false;
        doIntakePulse = false;
        shouldRemoveBalls = false;

        TurretSpinMotor
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setState(0);
        publicQueuer.executeNow(new StateAction(TurretAngle.states.DEFAULT));
        TurretRotateMotor.setState(0);
    };
    public void firingTurret(boolean shouldFire) {
        double distanceToWallOdometry = calculateDistanceToWallInMeters(ComplexFollower.getCurrentPose(), cfg.targetXRightPanel, cfg.targetYRightPanel);
        double rotationToWallOdometry = - calculateHeadingAdjustment(ComplexFollower.getCurrentPose(), Math.toDegrees(ComplexFollower.getCurrentPose().getHeading()), cfg.targetXRightPanel, cfg.targetYRightPanel);
        //rotationToWallOdometry += cfg.autoZoneAdderFar;

        if (shouldFire) {

            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry)/* + cfg.autoVelAdder*/;
            TurretSpinMotor
                    .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                    .setState(targetVelocity);


            double turretAngleVal = distanceToAngleFunction(distanceToWallOdometry);
            TurretAngle
                    .setState(turretAngleVal);

            if (shouldHoldTurretForCameraScan)
                rotationToWallOdometry = - calculateHeadingAdjustment(ComplexFollower.getCurrentPose(), Math.toDegrees(ComplexFollower.getCurrentPose().getHeading()), cfg.targetForCameraX, cfg.targetForCameraY);

            if (rotationToWallOdometry < 0) rotationToWallOdometry += 360;
            TurretRotateMotor
                .setState(rotationToWallOdometry)
            ;
        }
    }
    protected void HandleColors() {
        leftSensorColors = colorSensorLeft.getNormalizedColors();
        rightSensorColors = colorSensorRight.getNormalizedColors();

        Color.colorToHSV(leftSensorColors.toColor(), hsvLeftSensorColors);
        Color.colorToHSV(rightSensorColors.toColor(), hsvRightSensorColors);

        actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors,true);
        actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if (shouldResetRightSensorBall && resetLeftBallColorTimer.milliseconds() > 450) {
            shouldResetRightSensorBall = false;
            calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall;
        }


        if (!shouldRemoveBalls) {
            if (actualLeftSensorDetectedBall != BallColorSet_Decode.NoBall)
                calculatedLeftSensorDetectedBall = actualLeftSensorDetectedBall;

            if (actualRightSensorDetectedBall != BallColorSet_Decode.NoBall)
                calculatedRightSensorDetectedBall = actualRightSensorDetectedBall;
        }
        else {
            calculatedLeftSensorDetectedBall = actualLeftSensorDetectedBall;
            calculatedRightSensorDetectedBall = actualRightSensorDetectedBall;
        }

        if (actualLeftSensorDetectedBall == null) actualLeftSensorDetectedBall = BallColorSet_Decode.NoBall;
        if (actualRightSensorDetectedBall == null) actualRightSensorDetectedBall = BallColorSet_Decode.NoBall;

        if (calculatedLeftSensorDetectedBall == null) calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall;
        if (calculatedRightSensorDetectedBall == null) calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall;

        hasBallInLeftChamber = (calculatedLeftSensorDetectedBall != BallColorSet_Decode.NoBall);
        hasBallInRightChamber = (calculatedRightSensorDetectedBall != BallColorSet_Decode.NoBall);

        publicTelemetry.addData("LEFT_RED", (double)leftSensorColors.red * 10000.0 * leftSensorColorMultiplier);
        publicTelemetry.addData("LEFT_BLUE", (double)leftSensorColors.blue * 10000.0 * leftSensorColorMultiplier);
        publicTelemetry.addData("LEFT_GREEN", (double)leftSensorColors.green * 10000.0 * leftSensorColorMultiplier);

        publicTelemetry.addData("RIGHT_RED", (double)rightSensorColors.red * 10000.0);
        publicTelemetry.addData("RIGHT_BLUE", (double)rightSensorColors.blue * 10000.0);
        publicTelemetry.addData("RIGHT_GREEN", (double)rightSensorColors.green * 10000.0);

        publicTelemetry.addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
        publicTelemetry.addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
    }
    public static int lastGateState = 0;
    protected void pulseIntake(boolean shouldPulseIntake) {
        if (shouldPulseIntake) {
            doIntakePulse = false;
            publicQueuer.executeNow(new ActionSequence(
                    new DelayAction(50),
                    new StateAction(IntakeMotor.states.FULL_REVERSE),
                    new DelayAction(45),
                    new StateAction(IntakeMotor.states.FULL)
            ));
        }
    }
    public void makeConfig() {
        cfg = Config.getConfig("blue");
    }
    public void useCamera() {
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100);
        limelight3A.start();

        LLResult llResult = limelight3A.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            camId = fr.getFiducialId();
        }
        if (camId < 21 || camId > 23) camId = 23;
    }
    public void convertPoses() {
        starter = convertPose(starter);
        parkPose = convertPose(parkPose);
        small_triangle_shoot = convertPose(small_triangle_shoot);
        fininshHPCollectPose = convertPose(fininshHPCollectPose);
        secondZoneCameraCollect = convertPose(secondZoneCameraCollect);
        thirdZoneCameraCollect = convertPose(thirdZoneCameraCollect);
        thirdRowCollectDone = convertPose(thirdRowCollectDone);
        secondRowCollectDone = convertPose(secondRowCollectDone);
        firstRowCollectDone = convertPose(firstRowCollectDone);
        gateCollect = convertPose(gateCollect);
        tipBigTriangleShooting = convertPose(tipBigTriangleShooting);
        middleBigTriangleShooting = convertPose(middleBigTriangleShooting);
        middleBigTriangleShootingTurned90Deg = convertPose(middleBigTriangleShootingTurned90Deg);
        parkedBigTriangleShooting = convertPose(parkedBigTriangleShooting);
        autoCloseStart = convertPose(autoCloseStart);
        gateOpen = convertPose(gateOpen);
        bezierHelper1 = convertPose(bezierHelper1);
    }
    public Pose convertPose(Pose pose) {
        return pose;
    }
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose();
        return globalRobotPose;
    }
    public double getBallNumber() {

//        if (shouldSwitchChannel) {
//            limelight3A.pipelineSwitch(1);
//            shouldSwitchChannel = false;
//        }
        LLResult llResult = limelight3A.getLatestResult();
        limelight3A.captureSnapshot("HpSnap");

        if (llResult != null) {
            double[] pythonData = llResult.getPythonOutput();
            if (pythonData.length > 0) {
                double firstValue = pythonData[0];
                publicTelemetry.addData("Python Val 1", firstValue);
                return firstValue;
            }
        }
        return 0;
    }
    public void EmergencyOverrideAtTheEnd() {
        publicQueuer.clearQueue();
        publicQueuer.executeNow(new ActionSequence(
                new HoldAction(500),
                new GeneralAction(turnStuffOff)
        ));
    }
}