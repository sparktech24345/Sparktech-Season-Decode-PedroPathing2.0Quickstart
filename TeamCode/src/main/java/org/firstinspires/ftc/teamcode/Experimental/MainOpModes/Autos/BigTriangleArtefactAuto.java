package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftSensorColorMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalCamId;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.pose;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateDistanceToWallInMeters;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateHeadingAdjustment;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer1ForSorting;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer2ForSorting;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer3;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer4;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timerToCloseGate;

import static java.lang.Double.min;
import static java.lang.Math.max;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

import java.io.IOException;
import java.security.GeneralSecurityException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import kotlinx.coroutines.Delay;

@Config
@Autonomous(name = "Big Triangle Auto BLUE", group = "AAA")
public class BigTriangleArtefactAuto extends OpMode {
    public RobotController robot;
    private AutoRecorder recorder;
    private Limelight3A limelight3A;
    public static boolean doIntakePulse = false;
    public static MainConfig cfg;
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
    private Pose firstRowCollectDone = pose(77.5, 38 + 1.5, 90);
    private Pose gateCollect = pose(49.5, 47, 40);
    private Pose gateActualCollect = pose(50, 52, 60);
    private Pose gateHelperPoint = pose(30, 31, 55); // helper for the collect
    private Pose gateHold = pose(50.8, 44, 90); // not used
    private Pose tipBigTriangleShooting = pose(67, 0, 180);
    private Pose tipBigTriangleShootingTurned90Deg = pose(67, 0, 90);
    private Pose middleBigTriangleShooting = pose(87, 0, 180);
    private Pose middleBigTriangleShootingTurned90Deg = pose(87, 0, 90);
    private Pose parkedBigTriangleShooting = pose(100, 4.5, 180);
    private Pose gateOpen = pose(59, 43.5, 90); // actual gate opener
    private Pose gateOpenHelper = pose(48, 30, 90);
    private Pose gateSecond = pose(57, 30, 90);
    private Pose gateSecondOpen = pose(57, 39, 90);
    public static double distanceToWallOdometry;
    public static double rotationToWallOdometry;
    public static int camId = 23;
    public static boolean moveToZero = false;

    @Override
    public void init() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                controls();
                telemetry();
            }

            private void telemetry() {
                        RobotController.telemetry.addData("robot rotation", Math.toDegrees(robot.getCurrentPose().getHeading()));
                        RobotController.telemetry.addData("robot Y", robot.getCurrentPose().getY());
                        RobotController.telemetry.addData("robot X", robot.getCurrentPose().getX());
                        RobotController.telemetry.addData("current velocity",robot.getMotorComponent("TurretSpinMotor").getVelocity());
                        RobotController.telemetry.addData("is moving",isMoving);
                        RobotController.telemetry.addData("Intake Current",robot.getMotorComponent("IntakeMotor").getCurrent());
                        RobotController.telemetry.addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
                        RobotController.telemetry.addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
                        RobotController.telemetry.addData("Current cam id", camId);

            }

            private void controls() { // this will happen in a loop
                isMoving = ComplexFollower.instance().isBusy();
                if(shouldUseColorSensors) HandleColors();
                firingTurret(shouldFire);
                pulseIntake(doIntakePulse);
                if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(gateCollect) && !ComplexFollower.done()) ComplexFollower.interrupt();
                if (ComplexFollower.followingForMS() > 1000 && ComplexFollower.getTarget().equals(gateActualCollect) && !ComplexFollower.done()) ComplexFollower.interrupt();

                distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetXAutoClose, cfg.targetYAutoClose);
                rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetXAutoClose, cfg.targetYAutoClose);
                if(rotationToWallOdometry < 0) rotationToWallOdometry += 360;

                if (startAuto) {
                    startAuto = false;
                    if(shouldMakeAutoWithout3rdRow) makeLeverAutoWithout3rdRow();
                    else if(shouldMakeSortedAuto) makeSortedAuto();
                    else makeLeverAuto();
                    shouldMakeSortedAuto = false;
                    shouldMakeAutoWithout3rdRow = false;
                }
                if(timer.milliseconds() > 29000 + 800){
                    EmergencyOverrideAtTheEnd();
                    timer.reset();
                }
            }
        };
        makeConfig();
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
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
    public void init_loop() {
        //if(robot.getKey("A1").IsHeld) shouldMakeSortedAuto = true;
        robot.init_loop();
        robot.getTurretComponent("TurretRotateMotor").setTarget(cfg.rotationForInitClsoeZone);
        useCamera();
        RobotController.telemetry.addData("id: ",camId);
        RobotController.telemetry.addData("shouldMakeSortedAuto: ",shouldMakeSortedAuto);
        RobotController.telemetry.addData("shouldMakeAutoWithout3rdRow: ",shouldMakeAutoWithout3rdRow);
    }

    @Override
    public void start() {
        ComplexFollower.setPose(closeStarter);
        timer.reset();
        startAuto = true;
    }

    @Override
    public void loop() {
        recorder.update();
        robot.loop();
    }

    @Override
    public void stop() {
        try {
            recorder.save();
            passPose();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    /// =============================== Lever Auto ============================


    public void makeLeverAuto(){
        robot.addToQueue(
                /// prep and firing preload
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(() -> shouldFire = true),
                new GeneralAction(() -> shouldUseColorSensors = false),
//                new GeneralAction(() -> shouldBoostOnTheGoVelocityLogic = true),
//                new GeneralAction(() -> shouldBoostOnTheGoTurretLogic = true),
//                new GeneralAction(() -> shouldUseColorSensors = false),
//                new GeneralAction(() -> {
//                    robot.executeNow(new ActionSequence(
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
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff)
        );
    }



     ///  =============================== Make Sorted Auto ===============================

    public void makeSortedAuto(){
        robot.addToQueue(
                /// prep and firing preload
                new StateAction("IntakeMotor","FULL"),
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
                new HoldAction(gateOpen,1000),
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
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff)
        );
    }


    public void makeLeverAutoWithout3rdRow(){
        robot.addToQueue(
                /// prep and firing preload
                new StateAction("IntakeMotor","FULL"),
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

                new MoveAction(parkedBigTriangleShooting,false,BezierCurveTypes.ReverseTangentHeading,0),
                /// no more third row


                //shut up after parking
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff)
        );
    }




    /// Runnables
    public static double FAR_TARGET_VELOCITY = 1240;
    public static double FAR_TARGET_ANGLE = 280;
    public void firingTurret(boolean shouldFire) {
        if(shouldFire){
            // ----------------------- Power Stuff -----------------------

            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry);

            if(shouldBoostOnTheGoVelocityLogic) targetVelocity += velocityAdderOnTheGo;

            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                    .setTarget(targetVelocity);


            // ----------------------- Angle Stuff -----------------------

            double turretAngleVal = distanceToAngleFunction(distanceToWallOdometry);
            if(shouldBoostOnTheGoVelocityLogic) turretAngleVal = angleOnTheGo;
            robot.getServoComponent("TurretAngle")
                    .setTarget(turretAngleVal);

            // ----------------------- Rotation Stuff -----------------------
            if(shouldHoldTurretForClassifierScan)
                rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetForClassifierX, cfg.targetForClassifierY);

            if(shouldHoldTurretForClassifierScanNumber2)
                rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetForClassifierXNumber2, cfg.targetForClassifierYNumber2);

            if(shouldBoostOnTheGoVelocityLogic) rotationToWallOdometry += rotationOnTheGo;
            if(rotationToWallOdometry < 0) rotationToWallOdometry += 360;
            if(moveToZero) {
                rotationToWallOdometry = cfg.targetForFirstClassifierScan;
            }
            robot.getTurretComponent("TurretRotateMotor")
                    .setTarget(rotationToWallOdometry)
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
    public void firePPG(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // left right right
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("LeftGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence( // right right left
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green isnt or is in intake
                robot.executeNow(new ActionSequence( // right left right
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
        }
    }
    public void firePGP(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // left right left
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("LeftGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence( // right left right
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green ball is in intake
                robot.executeNow(new ActionSequence( // right right left
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
        }
    }
    public void fireGPP(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // right right left
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence( //left left right
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("LeftGateServo", "CLOSED"),
                        new DelayAction(timer1ForSorting),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timer2ForSorting),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green ball is in intake, cant actually sort this
                robot.executeNow(new ActionSequence(
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new DelayAction(timer3),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timer4),
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(1000),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
        }
    }
    Runnable fireUnsortedBalls = () -> {
        robot.executeNow(new ActionSequence(
                new StateAction("RightGateServo", "OPEN"),
                new DelayAction(timerToCloseGate),
                new StateAction("RightGateServo", "CLOSED"),
                new DelayAction(timer3),
                new StateAction("LeftGateServo", "OPEN"),
                new DelayAction(timer4),
                new StateAction("RightGateServo", "OPEN"),

                // close gates
                new DelayAction(1000),
                new StateAction("RightGateServo", "CLOSED"),
                new StateAction("LeftGateServo", "CLOSED")
        ));
    };
    Runnable turnStuffOff = () -> {
        shouldFire = false;

        robot.getMotorComponent("TurretSpinMotor")
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setTarget(0);
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT")); // go to default position
            robot.getTurretComponent("TurretRotateMotor").setTarget(0);
            robot.executeNow(new StateAction("IntakeMotor","OFF"));
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

        RobotController.telemetry.addData("LEFT_RED", (double)leftSensorColors.red * 10000.0 * leftSensorColorMultiplier);
        RobotController.telemetry.addData("LEFT_BLUE", (double)leftSensorColors.blue * 10000.0 * leftSensorColorMultiplier);
        RobotController.telemetry.addData("LEFT_GREEN", (double)leftSensorColors.green * 10000.0 * leftSensorColorMultiplier);

        RobotController.telemetry.addData("RIGHT_RED", (double)rightSensorColors.red * 10000.0);
        RobotController.telemetry.addData("RIGHT_BLUE", (double)rightSensorColors.blue * 10000.0);
        RobotController.telemetry.addData("RIGHT_GREEN", (double)rightSensorColors.green * 10000.0);

        RobotController.telemetry.addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
        RobotController.telemetry.addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
    }
    public static int lastGateState = 1;
    public void makeConfig(){
    cfg = new MainConfig(MainConfig.Configs.Blue);
}

    protected void pulseIntake(boolean shouldPulseIntake){
        if(shouldPulseIntake){
            doIntakePulse = false;
            robot.executeNow(new ActionSequence(
                    new DelayAction(80),
                    new StateAction("IntakeMotor","FULL_REVERSE"),
                    new DelayAction(50),
                    new StateAction("IntakeMotor","FULL")
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
    public void useCamera(){
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
        if(camId < 21 || camId > 23) camId = 23;
        passMotif();
    }
    public Pose convertPose(Pose pose){
        return pose;
    }
    public int passMotif(){
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


                maxX = min(maxX,min(x1,x2));
                maxY = min(maxY,min(y1,y2));

                RobotController.telemetry.addData("Ball Corners No." + index, x1 + "  " + y1 + " ::: " + x2 + "  " + y2);


            }

            //detectedBalls = max(detectedBalls,countedBalls);
//            detectedBalls = countedBalls;
            RobotController.telemetry.addData("BALLS LIST", ball_colors);




            RobotController.telemetry.addData("maxX", maxX);
            RobotController.telemetry.addData("maxY", maxY);

            if(shouldHoldTurretForClassifierScan || moveToZero) {
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
            else if(shouldHoldTurretForClassifierScanNumber2){

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
        RobotController.telemetry.addData("Counted Balls", countedBalls);
        RobotController.telemetry.addData("Calculated Balls", detectedBalls);
        limelight3A.captureSnapshot("Classifier scan" + timer.milliseconds());
    };
    public Runnable countBallsInClassifierWithDelay = () -> {
        robot.executeNow(new ActionSequence(
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

        if(camId < 21) camId += 3;

        //backup
        if(camId > 23) camId -= 3;
    };
    public Runnable processCameraScanning = () -> {
        robot.executeNow(new GeneralAction(processClassifierAndSwitchMotif)
//                new ActionSequence(
//                        new DelayAction(500),
//
//                )
        );
    };
    public void methodToOverWrite(){
    }
    public void EmergencyOverrideAtTheEnd(){
        robot.clearMainQueue();
        robot.executeNow(new ActionSequence(
                new HoldAction(500),
                new GeneralAction(turnStuffOff)
        ));
    }
}