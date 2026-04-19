package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Components.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.*;

import static java.lang.Double.min;
import static java.lang.Math.max;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BezierCurveTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.Config;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@com.acmerobotics.dashboard.config.Config
@Autonomous(name = "Big Triangle Auto BLUE", group = "AAA")
public class BigTriangleArtefactAuto extends ComplexOpMode {
    private AutoRecorder recorder;
    private Limelight3A limelight3A;
    public static boolean doIntakePulse = false;
    public static Config cfg;
    private boolean startAuto = false;
    public static boolean isMoving;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime movingTimer = new ElapsedTime();
    ElapsedTime isFiringTimer = new ElapsedTime();
    ElapsedTime shouldJiggle = new ElapsedTime();
    ElapsedTime shouldFinish = new ElapsedTime();
    private boolean had_balls = false;
    private int collectNumber = 0;
    public int detectedBalls;

    /// ----------------- Color Sensor Stuff ------------------
    protected NormalizedColorSensor colorSensorRight;
    protected NormalizedColorSensor colorSensorLeft;
    protected NormalizedRGBA rightSensorColors;
    protected NormalizedRGBA leftSensorColors;
    final float[] hsvRightSensorColors = new float[3];
    final float[] hsvLeftSensorColors = new float[3];
    public static int ballCounter = 0;
    protected BallColorSet_Decode actualRightSensorDetectedBall;
    protected BallColorSet_Decode calculatedRightSensorDetectedBall;
    protected BallColorSet_Decode actualLeftSensorDetectedBall;
    protected BallColorSet_Decode calculatedLeftSensorDetectedBall;
    protected BallColorSet_Decode ballToFire;
    public static boolean hasBallInRightChamber = false;
    public static boolean hadBallInRightChamberInPast = false;
    public static boolean hasBallInLeftChamber = false;
    public static boolean hadBallInLeftChamberInPast = false;
    public static boolean shouldRemoveBalls = false;
    public static boolean shouldBoostOnTheGoVelocityLogic = false;
    public static boolean shouldBoostOnTheGoTurretLogic = false;
    public static boolean shouldUseColorSensors = false;
    public static double velocityAdderOnTheGo = 100;
    public static double rotationOnTheGo = 3.5;
    public static double angleOnTheGo = 140; // old -75
    BallColorQueue ballColorQueue = new BallColorQueue();
    public static boolean shouldFire = false;
    public static boolean shouldMakeSortedAuto = false;
    public static boolean shouldMakeAutoWithout3rdRow = false;
    public static boolean shouldHoldTurretForClassifierScan = false;
    public static boolean shouldHoldTurretForClassifierScanNumber2 = false;

    /// --------------------------------------------------------
    private Pose closeStarter = pose(119, 30.5, 180); // would also be around 1.4x
    private Pose small_triangle_shoot = pose(-1.5, 12.5, 90);
    private Pose parkPose = pose(-2, 26.5, 90);
    private Pose fininshHPCollectPose = pose(-1.2,48.5,90); // hp collect
    private Pose secondZoneCameraCollect = pose(14.8, 48.5, 90);
    private Pose thirdZoneCameraCollect = pose(30.96, 48.5, 90);
    private Pose thirdRowCollectDone = pose(27, 45, 90); // third row done
    private Pose secondRowCollectDone = pose(48.7, 43.5 + 1.5, 90);
    private Pose firstRowCollectDone = pose(77.5, 38 + 2, 90);
    private Pose gateCollect = pose(49.5, 47, 40);
    private Pose gateActualCollect = pose(50, 52, 60);
    private Pose gateHelperPoint = pose(30, 31, 55); // helper for the collect
    private Pose gateHold = pose(50.8, 44, 90); // not used
    private Pose tipBigTriangleShooting = pose(67, 0, 180);
    private Pose tipBigTriangleShootingTurned90Deg = pose(67, 0, 90);
    private Pose middleBigTriangleShooting = pose(87, 0, 180);
    private Pose middleBigTriangleShootingTurned90Deg = pose(87, 0, 90);
    private Pose parkedBigTriangleShooting = pose(100, 4.5, 180);
    private Pose gateOpen = pose(59, 44, 90); // actual gate opener
    private Pose gateOpenHelper = pose(48, 30, 90);
    private Pose gateSecond = pose(57, 30, 90);
    private Pose gateSecondOpen = pose(57, 39, 90);
    public static double distanceToWallOdometry;
    public static double rotationToWallOdometry;
    public static int camId = 23;
    public static boolean moveToZero = false;

    @Override
    public void telemetry() {
        publicTelemetry.addData("robot rotation", Math.toDegrees(ComplexFollower.getCurrentPose().getHeading()));
        publicTelemetry.addData("robot Y", ComplexFollower.getCurrentPose().getY());
        publicTelemetry.addData("robot X", ComplexFollower.getCurrentPose().getX());
        publicTelemetry.addData("current velocity", TurretSpinMotor.getVelocity());
        publicTelemetry.addData("is moving",isMoving);
        publicTelemetry.addData("Intake Current",IntakeMotor.getCurrent());
        publicTelemetry.addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
        publicTelemetry.addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
        publicTelemetry.addData("Current cam id", camId);

    }

    @Override
    public void update() {
        recorder.update();
        isMoving = ComplexFollower.instance().isBusy();
        if (shouldUseColorSensors) HandleColors();
        firingTurret(shouldFire);
        pulseIntake(doIntakePulse);
        if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(gateCollect) && !ComplexFollower.done()) ComplexFollower.interrupt();
        if (ComplexFollower.followingForMS() > 1000 && ComplexFollower.getTarget().equals(gateActualCollect) && !ComplexFollower.done()) ComplexFollower.interrupt();

        distanceToWallOdometry = calculateDistanceToWallInMeters(ComplexFollower.getCurrentPose(), cfg.targetXAutoClose, cfg.targetYAutoClose);
        rotationToWallOdometry = - calculateHeadingAdjustment(ComplexFollower.getCurrentPose(), Math.toDegrees(ComplexFollower.getCurrentPose().getHeading()), cfg.targetXAutoClose, cfg.targetYAutoClose);
        if (rotationToWallOdometry < 0) rotationToWallOdometry += 360;

        if (startAuto) {
            startAuto = false;
            if (shouldMakeAutoWithout3rdRow) makeLeverAutoWithout3rdRow();
            else if (shouldMakeSortedAuto) makeSortedAuto();
            else makeLeverAuto();
            shouldMakeSortedAuto = false;
            shouldMakeAutoWithout3rdRow = false;
        }
        if (timer.milliseconds() > 29000 + 800) {
            EmergencyOverrideAtTheEnd();
            timer.reset();
        }
    }

    @Override
    public void initialize() {
        makeConfig();
        Components.init();
        recorder = new AutoRecorder();
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        shouldFire = false; lastGateState = 1;
        hadBallInRightChamberInPast = false; hadBallInLeftChamberInPast = false;
        doIntakePulse = false;
        shouldMakeSortedAuto = false;
        shouldMakeAutoWithout3rdRow = false;
        shouldHoldTurretForClassifierScan = false;
        collectNumber = 0;
        movingTimer.reset();
        convertPoses();
        methodToOverWrite();
    }

    @Override
    public void init_update() {
        //if (ComplexGamepad.A1.get(o.IsHeld) shouldMakeSortedAuto = true;
        TurretRotateMotor.setState(cfg.rotationForInitCloseZone);
        useCamera();
        publicTelemetry.addData("id: ", camId);
        publicTelemetry.addData("shouldMakeSortedAuto: ", shouldMakeSortedAuto);
        publicTelemetry.addData("shouldMakeAutoWithout3rdRow: ", shouldMakeAutoWithout3rdRow);
    }

    @Override
    public void on_start() {
        ComplexFollower.setPose(closeStarter);
        timer.reset();
        startAuto = true;
    }

    @Override
    public void on_stop() {
        try {
            recorder.save();
            passPose();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    /// =============================== Lever Auto ============================


    public void makeLeverAuto() {
        publicQueuer.addToQueue(
                /// prep and firing preload
                new StateAction(IntakeMotor.states.FULL),
                new GeneralAction(() -> shouldFire = true),
                new GeneralAction(() -> shouldUseColorSensors = false),
//                new GeneralAction(() -> shouldBoostOnTheGoVelocityLogic = true),
//                new GeneralAction(() -> shouldBoostOnTheGoTurretLogic = true),
//                new GeneralAction(() -> shouldUseColorSensors = false),
//                new GeneralAction(() -> {
//                    publicQueuer.executeNow(new ActionSequence(
//                            new DelayAction(800),
//                            new GeneralAction(fireUnsortedBalls),
//                            new DelayAction(1000),
//                            new GeneralAction(() -> shouldBoostOnTheGoVelocityLogic = false),
//                            new DelayAction(1200),
//                            new GeneralAction(() -> shouldBoostOnTheGoTurretLogic  = false)
//                    ));
//                }),
                new MoveAction(middleBigTriangleShooting),//false,BezierCurveTypes.TangentHeading,0),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                /// finished preload and path



                /// second row
                new GeneralAction(() -> collectNumber++),
                //new MoveAction(second_row_ready),
                new MoveAction(secondRowCollectDone,true,BezierCurveTypes.TangentHeading,0),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                /// end of second row firing


                /// collecting lever pose
                new GeneralAction(() -> collectNumber++),
                new MoveAction(gateCollect,BezierCurveTypes.LinearHeading,0,gateHelperPoint),
                new HoldAction(gateActualCollect,1400), //1200
                //new DelayAction(350),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                ///finished gate collect


                /// collecting lever pose number 2
                new GeneralAction(() -> collectNumber++),
                new MoveAction(gateCollect,BezierCurveTypes.LinearHeading,0,gateHelperPoint),
                new HoldAction(gateActualCollect,1800), //1600
                //new DelayAction(800),
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                new GeneralAction(() -> shouldUseColorSensors = false),
                ///finished gate collect number 2


                /// collecting the first row
                new GeneralAction(() -> collectNumber++),
                new MoveAction(firstRowCollectDone,true,BezierCurveTypes.LinearHeading,0),
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new MoveAction(tipBigTriangleShooting),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                new GeneralAction(() -> shouldUseColorSensors = false),
                /// end of first row firing


                /// beginning of third row collect
                new GeneralAction(() -> collectNumber++),
                new MoveAction(thirdRowCollectDone,true,BezierCurveTypes.TangentHeading,0),
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new MoveAction(parkedBigTriangleShooting,false,BezierCurveTypes.ReverseTangentHeading,0),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                new GeneralAction(() -> shouldUseColorSensors = false),
                ///end of third row firing

                //shut up after parking
                new StateAction(IntakeMotor.states.OFF),
                new GeneralAction(turnStuffOff)
        );
    }



     ///  =============================== Make Sorted Auto ===============================

    public void makeSortedAuto() {
        publicQueuer.addToQueue(
                /// prep and firing preload
                new StateAction(IntakeMotor.states.FULL),
                new GeneralAction(() -> shouldFire = true),
                new GeneralAction(() -> shouldUseColorSensors = false),
                new MoveAction(middleBigTriangleShooting),
//                new DelayAction(200),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                /// finished preload and path


                new GeneralAction(() -> limelight3A.pipelineSwitch(8)), // preactivly switch pipeline

                /// second row
                new GeneralAction(() -> collectNumber++),
                new MoveAction(secondRowCollectDone,true,BezierCurveTypes.TangentHeading,0),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                /// end of second row firing


                /// collecting lever pose
                new GeneralAction(() -> collectNumber++),
                new MoveAction(gateCollect,BezierCurveTypes.LinearHeading,0,gateHelperPoint),
                new HoldAction(gateActualCollect,1600), //1200
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                // gate holding comes here
//                new MoveAction(gateOpenHelper),
//                new MoveAction(gateOpen),
                new MoveAction(gateSecond),
//                new DelayAction(400),
                new HoldAction(gateOpen,1200),
//                new MoveAction(gateSecondOpen),
//                new DelayAction(700),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireSortedBalls),
                new DelayAction(1800),
                new GeneralAction(() -> shouldUseColorSensors = false),
                ///finished gate collect


                new GeneralAction(() -> limelight3A.pipelineSwitch(9)), // preactivly switch pipeline


                /// collecting the first row
                new GeneralAction(() -> collectNumber++),
                new MoveAction(firstRowCollectDone,true,BezierCurveTypes.LinearHeading,0),
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new GeneralAction(() -> moveToZero = true), // right now using this
                new GeneralAction(() -> shouldHoldTurretForClassifierScan = false),
                new DelayAction(200),
                new GeneralAction(countBallsInClassifierWithDelay),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new DelayAction(200),
                new GeneralAction(processCameraScanning),
                new GeneralAction(() -> shouldHoldTurretForClassifierScan = false),
                new GeneralAction(() -> moveToZero = false),
                new DelayAction(200),
                new GeneralAction(fireSortedBalls),
                new DelayAction(1800),
                new GeneralAction(() -> shouldUseColorSensors = false),
                /// end of first row firing


                /// beginning of third row collect
                new GeneralAction(() -> collectNumber++),
                new MoveAction(thirdRowCollectDone,true,BezierCurveTypes.TangentHeading,0),
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new MoveAction(parkedBigTriangleShooting,false,BezierCurveTypes.ReverseTangentHeading,0),
                new GeneralAction(() -> shouldHoldTurretForClassifierScanNumber2 = true),
                new DelayAction(500),
                new GeneralAction(countBallsInClassifier),
                new DelayAction(300),
                new GeneralAction(processCameraScanning),
                new DelayAction(250),
                new GeneralAction(() -> shouldHoldTurretForClassifierScanNumber2 = false),
                new GeneralAction(fireSortedBalls),
                new DelayAction(1800),
                new GeneralAction(() -> shouldUseColorSensors = false),
                ///end of third row firing

                //shut up after parking
                new StateAction(IntakeMotor.states.OFF),
                new GeneralAction(turnStuffOff)
        );
    }


    public void makeLeverAutoWithout3rdRow() {
        publicQueuer.addToQueue(
                /// prep and firing preload
                new StateAction(IntakeMotor.states.FULL),
                new GeneralAction(() -> shouldFire = true),
                new GeneralAction(() -> shouldUseColorSensors = false),
                new MoveAction(middleBigTriangleShooting),//false,BezierCurveTypes.TangentHeading,0),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                /// finished preload and path



                /// second row
                new GeneralAction(() -> collectNumber++),
                //new MoveAction(second_row_ready),
                new MoveAction(secondRowCollectDone,true,BezierCurveTypes.TangentHeading,0),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                /// end of second row firing


                /// collecting lever pose
                new GeneralAction(() -> collectNumber++),
                new MoveAction(gateCollect,BezierCurveTypes.LinearHeading,0,gateHelperPoint),
                new HoldAction(gateActualCollect,1400), //1200
                //new DelayAction(350),
                new GeneralAction(() -> doIntakePulse = true),

                /// give balls cuz u have third row stuff
                new MoveAction(gateSecond),
                new HoldAction(gateOpen,1000),

                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                ///finished gate collect


                /// collecting lever pose number 2
                new GeneralAction(() -> collectNumber++),
                new MoveAction(gateCollect,BezierCurveTypes.LinearHeading,0,gateHelperPoint),
                new HoldAction(gateActualCollect,2100), //1800
                //new DelayAction(800),
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                new GeneralAction(() -> shouldUseColorSensors = false),
                ///finished gate collect number 2


                /// collecting the first row
                new GeneralAction(() -> collectNumber++),
                new MoveAction(firstRowCollectDone,true,BezierCurveTypes.LinearHeading,0),
                new GeneralAction(() -> doIntakePulse = true),
                new GeneralAction(() -> shouldUseColorSensors = true),
                //new MoveAction(tipBigTriangleShooting),
                new GeneralAction(() -> shouldUseColorSensors = false),
                /// end of first row firing

                new MoveAction(parkedBigTriangleShooting,false,BezierCurveTypes.ReverseTangentHeading,0),
                ///firing here
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
                /// no more third row


                //shut up after parking
                new StateAction(IntakeMotor.states.OFF),
                new GeneralAction(turnStuffOff)
        );
    }




    /// Runnables
    public static double FAR_TARGET_VELOCITY = 1240;
    public static double FAR_TARGET_ANGLE = 280;
    public void firingTurret(boolean shouldFire) {
        if (shouldFire) {
            // ----------------------- Power Stuff -----------------------

            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry);

            if (shouldBoostOnTheGoVelocityLogic) targetVelocity += velocityAdderOnTheGo;

            TurretSpinMotor
                    .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                    .setState(targetVelocity);


            // ----------------------- Angle Stuff -----------------------

            double turretAngleVal = distanceToAngleFunction(distanceToWallOdometry);
            if (shouldBoostOnTheGoVelocityLogic) turretAngleVal = angleOnTheGo;
            TurretAngle
                    .setState(turretAngleVal);

            // ----------------------- Rotation Stuff -----------------------
            if (shouldHoldTurretForClassifierScan)
                rotationToWallOdometry = - calculateHeadingAdjustment(ComplexFollower.getCurrentPose(), Math.toDegrees(ComplexFollower.getCurrentPose().getHeading()), cfg.targetForClassifierX, cfg.targetForClassifierY);

            if (shouldHoldTurretForClassifierScanNumber2)
                rotationToWallOdometry = - calculateHeadingAdjustment(ComplexFollower.getCurrentPose(), Math.toDegrees(ComplexFollower.getCurrentPose().getHeading()), cfg.targetForClassifierXNumber2, cfg.targetForClassifierYNumber2);

            if (shouldBoostOnTheGoVelocityLogic) rotationToWallOdometry += rotationOnTheGo;
            if (rotationToWallOdometry < 0) rotationToWallOdometry += 360;
            if (moveToZero) {
                rotationToWallOdometry = cfg.targetForFirstClassifierScan;
            }
            TurretRotateMotor
                    .setState(rotationToWallOdometry)
            ;
            return;
        }
    }
    Runnable fireSortedBalls = () -> {
        switch (camId) {
            case 23: // ppg
                firePPG();
//                ballColorQueue.add(BallColorSet_Decode.Purple);
//                ballColorQueue.add(BallColorSet_Decode.Purple);
//                ballColorQueue.add(BallColorSet_Decode.Green);
                break;

            case 22: // pgp
                firePGP();
//                ballColorQueue.add(BallColorSet_Decode.Purple);
//                ballColorQueue.add(BallColorSet_Decode.Green);
//                ballColorQueue.add(BallColorSet_Decode.Purple);
                break;

            case 21: // gpp
                fireGPP();
//                ballColorQueue.add(BallColorSet_Decode.Green);
//                ballColorQueue.add(BallColorSet_Decode.Purple);
//                ballColorQueue.add(BallColorSet_Decode.Purple);
                break;
        }
    };
    public void firePPG() {
        int greenBallPosition = 3;
        if (calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                publicQueuer.executeNow(new ActionSequence( // left right right
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(LeftGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 2: // if green is on the left
                publicQueuer.executeNow(new ActionSequence( // right right left
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 3: // green isnt or is in intake
                publicQueuer.executeNow(new ActionSequence( // right left right
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
        }
    }
    public void firePGP() {
        int greenBallPosition = 3;
        if (calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                publicQueuer.executeNow(new ActionSequence( // left right left
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(LeftGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 2: // if green is on the left
                publicQueuer.executeNow(new ActionSequence( // right left right
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 3: // green ball is in intake
                publicQueuer.executeNow(new ActionSequence( // right right left
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
        }
    }
    public void fireGPP() {
        int greenBallPosition = 3;
        if (calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                publicQueuer.executeNow(new ActionSequence( // right right left
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 2: // if green is on the left
                publicQueuer.executeNow(new ActionSequence( //left left right
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(LeftGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 3: // green ball is in intake, cant actually sort this
                publicQueuer.executeNow(new ActionSequence(
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
                break;
        }
    }
    Runnable fireUnsortedBalls = () -> {
        publicQueuer.executeNow(new ActionSequence(
                new StateAction(RightGateServo.states.OPEN),
                new DelayAction(timerToCloseGate),
                new StateAction(RightGateServo.states.CLOSED),
                new DelayAction(timer3),
                new StateAction(LeftGateServo.states.OPEN),
                new DelayAction(timer4),
                new StateAction(RightGateServo.states.OPEN),

                // close gates
                new DelayAction(1000),
                new StateAction(RightGateServo.states.CLOSED),
                new StateAction(LeftGateServo.states.CLOSED)
        ));
    };
    Runnable turnStuffOff = () -> {
        shouldFire = false;

        TurretSpinMotor
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setState(0);
        publicQueuer.executeNow(new StateAction(TurretAngle.states.DEFAULT)); // go to default position
        TurretRotateMotor.setState(0);
        publicQueuer.executeNow(new StateAction(IntakeMotor.states.OFF));
    };

    protected void HandleColors() {
        leftSensorColors = colorSensorLeft.getNormalizedColors();
        rightSensorColors = colorSensorRight.getNormalizedColors();

        Color.colorToHSV(leftSensorColors.toColor(), hsvLeftSensorColors);
        Color.colorToHSV(rightSensorColors.toColor(), hsvRightSensorColors);

        actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors,true);
        actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if (!shouldRemoveBalls) { // when not moving balls out of chambers they dont have permission to change to no ball
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
    public static int lastGateState = 1;
    public void makeConfig() {
    cfg = Config.getConfig("blue");
}

    protected void pulseIntake(boolean shouldPulseIntake) {
        if (shouldPulseIntake) {
            doIntakePulse = false;
            publicQueuer.executeNow(new ActionSequence(
                    new DelayAction(80),
                    new StateAction(IntakeMotor.states.FULL_REVERSE),
                    new DelayAction(50),
                    new StateAction(IntakeMotor.states.FULL)
            ));
        }
    }
    public void convertPoses() {
        closeStarter = convertPose(closeStarter);
        parkPose = convertPose(parkPose);
        small_triangle_shoot = convertPose(small_triangle_shoot);
        fininshHPCollectPose = convertPose(fininshHPCollectPose);
        secondZoneCameraCollect = convertPose(secondZoneCameraCollect);
        thirdZoneCameraCollect = convertPose(thirdZoneCameraCollect);
        thirdRowCollectDone = convertPose(thirdRowCollectDone);
        secondRowCollectDone = convertPose(secondRowCollectDone);
        firstRowCollectDone = convertPose(firstRowCollectDone);
        gateCollect = convertPose(gateCollect);
        gateActualCollect = convertPose(gateActualCollect);
        gateHelperPoint = convertPose(gateHelperPoint);
        gateHold = convertPose(gateHold);
        tipBigTriangleShooting = convertPose(tipBigTriangleShooting);
        middleBigTriangleShooting = convertPose(middleBigTriangleShooting);
        tipBigTriangleShootingTurned90Deg = convertPose(tipBigTriangleShootingTurned90Deg);
        middleBigTriangleShootingTurned90Deg = convertPose(middleBigTriangleShootingTurned90Deg);
        parkedBigTriangleShooting = convertPose(parkedBigTriangleShooting);
        gateOpen = convertPose(gateOpen);
        gateOpenHelper = convertPose(gateOpenHelper);
        gateSecond = convertPose(gateSecond);
    }
    public void useCamera() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100); // poll 100 times per second
        limelight3A.start();

        LLResult llResult = limelight3A.getLatestResult();
        llResult.getFiducialResults();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            camId = fr.getFiducialId();
        }
        if (camId < 21 || camId > 23) camId = 23;
        passMotif ();
    }
    public Pose convertPose(Pose pose) {
        return pose;
    }
    public int passMotif () {
        globalCamId = camId;
        return camId;
    }
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose();
        return globalRobotPose;
    }
    public Runnable countBallsInClassifier = () -> {
        List<String> ball_colors = new ArrayList<>();
        int countedBalls = 0;
        LLResult result = limelight3A.getLatestResult();


        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            int index = 0;
            // counting corners stuff
            double maxX=999,maxY=999;
            double x1=0,x2=0,y1=0,y2=0;
            int counter = 0;
            List<List<Double>> listWithStuff;

            for (LLResultTypes.DetectorResult detection : detections) {
                ball_colors.add(detection.getClassName());
                if (index >= 8) break;
                switch (detection.getClassName()) {
                    case "purple":
                    case "green": {
                        countedBalls++;
                        break;
                    }
                    default:
                        break;
                }
                index++;


                listWithStuff = detection.getTargetCorners(); // List<List<x,y>>

                x1 = listWithStuff.get(0).get(0); // x
                y1 = listWithStuff.get(0).get(1); // y
                x2 = listWithStuff.get(1).get(0); // x
                y2 = listWithStuff.get(1).get(1); // y


                maxX = min(maxX, min(x1, x2));
                maxY = min(maxY, min(y1, y2));

                publicTelemetry.addData("Ball Corners No." + index, x1 + "  " + y1 + " ::: " + x2 + "  " + y2);


            }

            //detectedBalls = max(detectedBalls,countedBalls);
//            detectedBalls = countedBalls;
            publicTelemetry.addData("BALLS LIST", ball_colors);




            publicTelemetry.addData("maxX", maxX);
            publicTelemetry.addData("maxY", maxY);

            if (shouldHoldTurretForClassifierScan || moveToZero) {
                if (maxY == 0) detectedBalls = 0; // y este cu susul in jos sus = 0 jos = max
                else if (maxY >= 265) detectedBalls = 1;
                else if (maxY >= 247) detectedBalls = 2;
                else if (maxY >= 230) detectedBalls = 3;
                else if (maxY >= 215) detectedBalls = 4;
                else if (maxY >= 200) detectedBalls = 5;
                else if (maxY >= 180) detectedBalls = 6;
                else if (maxY >= 165) detectedBalls = 7;
                else detectedBalls = 8; // cant detect more then 8 anyway
            }
            else if (shouldHoldTurretForClassifierScanNumber2) {

                if (maxY == 0) detectedBalls = 0; // y este cu susul in jos sus = 0 jos = max
                else if (maxY >= 270) detectedBalls = 1;
                else if (maxY >= 255) detectedBalls = 2;
                else if (maxY >= 230) detectedBalls = 3;
                else if (maxY >= 222) detectedBalls = 4;
                else if (maxY >= 205) detectedBalls = 5;
                else if (maxY >= 180) detectedBalls = 6;
                else if (maxY >= 165) detectedBalls = 7;
                else detectedBalls = 8; // cant detect more then 8 anyway
            }
            else detectedBalls = countedBalls;

        }
        publicTelemetry.addData("Counted Balls", countedBalls);
        publicTelemetry.addData("Calculated Balls", detectedBalls);
        limelight3A.captureSnapshot("Classifier scan" + timer.milliseconds());
    };
    public Runnable countBallsInClassifierWithDelay = () -> {
        publicQueuer.executeNow(new ActionSequence(
                new DelayAction(1000),
                new GeneralAction(countBallsInClassifier)
        ));
    };
    public Runnable processClassifierAndSwitchMotif = () -> {
        //int detectedBalls = countBallsInClassifier();
        // only between 0-2
        int placesToShift = detectedBalls % 3;

        // 23 ppg, 22 pgp, 21 gpp, shift the needed amount
        camId = globalCamId - placesToShift;

        if (camId < 21) camId += 3;

        //backup
        if (camId > 23) camId -= 3;
    };
    public Runnable processCameraScanning = () -> {
        publicQueuer.executeNow(new GeneralAction(processClassifierAndSwitchMotif)
//                new ActionSequence(
//                        new DelayAction(500),
//
//                )
        );
    };
    public void methodToOverWrite() {
    }
    public void EmergencyOverrideAtTheEnd() {
        publicQueuer.clearQueue();
        publicQueuer.executeNow(new ActionSequence(
                new HoldAction(500),
                new GeneralAction(turnStuffOff)
        ));
    }
}