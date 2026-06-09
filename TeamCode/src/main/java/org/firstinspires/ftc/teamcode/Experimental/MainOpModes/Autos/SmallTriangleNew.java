package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.camTargetX;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.camTargetY;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.ballInAirTime;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateDistanceToWallInMeters;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateHeadingAdjustment;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.cameraImaginaryX;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.mainTimerForSorting;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer2;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer3;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer4;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timerBothOnOneChannelTimerForSorting;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timerToCloseGate;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.vMultiplier;

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
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BezierCurveTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

import java.util.List;

@Config
@Autonomous(name = "\uD83D\uDD35 Small Triangle Auto BLUE", group = "AAA") // 🔵
public class SmallTriangleNew extends OpMode {
    private RobotController robot;
    //private AutoRecorder recorder;
    public Limelight3A limelight3A;
    public static MainConfig cfg;
    private boolean shouldFire;
    public static boolean isMoving;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime resetLeftBallColorTimer = new ElapsedTime();
    public static boolean doIntakePulse = false;
    public static boolean shouldHoldTurretForCameraScan = false;
    public static boolean shouldCheckColorSensors = false;
    public static boolean doOnce = false;
    private static boolean shouldToAirSort = false;
    public static boolean shouldFireUnsortedBalls = false;
    public static double cameraAngleOverite = 0;

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
    public static boolean hasBallInIntake = false;
    public static boolean hasBallInRightChamber = false;
    public static boolean hasBallInLeftChamber = false;
    public static boolean shouldRemoveBalls = false;
    private boolean shouldResetRightSensorBall = false;
    public static boolean shouldPullFromQueue = false;
    BallColorQueue ballColorQueue = new BallColorQueue();

    /// distance sensor stuff
    private AnalogInput laserAnalog;

    /// other stuff

    public static double velocity = 1480;
    public static double angle = 266;
    public static double rotationNeededForCameraScan = 13; // about 13 degrees to scan with limelight
    public static int camId =23;
    public static int wentTooNumber2 =23;
    private Pose starter = pose( -0.7, 13.4, 90); // would also be around 1.4x
    private Pose small_triangle_shoot = pose(1.6, 9.5, 90);
    private Pose small_triangle_shootForSpecial = pose(5, 2, 90);
    private Pose parkPose = pose(1, 23.5, 90);
    private Pose fininshHPCollectPose = pose(0.5,44,90); // hp collect
    private Pose fininshHPCollectPoseNEW = pose(0.5,44,90); // hp collect
    private Pose secondZoneCameraCollect = pose(19 + 1, 44, 90); /// CHECK THIS slightly more up spot
    private Pose thirdZoneCameraCollect = pose(33.96, 44, 90);
    private Pose thirdRowCollectDone = pose(30, 42, 90); // third row done
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
    private Pose bezierHelper2 = pose(22, -12, 75);

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
            }
            private void controls() {
                isMoving = ComplexFollower.instance().isBusy();
                if(shouldCheckColorSensors) handleColors();
                checkToFireUnsortedBalls(shouldFireUnsortedBalls);
                if (ComplexFollower.followingForMS() > 1800 && ComplexFollower.getTarget().equals(fininshHPCollectPose) && !ComplexFollower.done()) ComplexFollower.interrupt();
                if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(fininshHPCollectPoseNEW) && !ComplexFollower.done()) ComplexFollower.interrupt();
                if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(small_triangle_shoot) && !ComplexFollower.done()) ComplexFollower.interrupt();
                cameraStuffUpdates(cfg.targetForCameraX,cfg.targetForCameraY);
                firingTurret(shouldFire);
                pulseIntake(doIntakePulse);
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
        //recorder = new AutoRecorder();
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        laserAnalog = hardwareMap.get(AnalogInput.class, distanceSensorName);
        convertPoses();
        shouldFire = false; lastGateState = 0; resetLeftBallColorTimer.reset();
        shouldRemoveBalls = false;shouldResetRightSensorBall = false; doIntakePulse = false;
        shouldFireUnsortedBalls = false;
        wentTooNumber2  =0;

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        doOnce = true;
    }

    @Override
    public void init_loop() {
        if(doOnce) ComplexFollower.resetAndInit(true);
        doOnce = false;
        robot.init_loop();
        robot.getTurretComponent("TurretRotateMotor").setTarget(cfg.rotationForInitSmallTriangle);
        double middleOfTheFieldY =0;
        if(currentTeamColor == TeamColor.Blue) middleOfTheFieldY = -16; else middleOfTheFieldY = 16;
        cameraStuffUpdates(cameraImaginaryX,middleOfTheFieldY);
        useCamera();
        RobotController.telemetry.addData("id", camId);
    }

    @Override
    public void start() {
        ComplexFollower.setPose(starter);
        timer.reset();
        makeAuto();
    }

    @Override
    public void loop() {
        //recorder.update();
        robot.loop();
    }

    @Override
    public void stop() {
        //recorder.save();
        passPose();
    }
    private void makeAuto() {
        robot.addToQueue(
                // preload
                new GeneralAction(() -> shouldFire = true), // prep outtake
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(900), // revving up outtake
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // end of preload

                //first hp collect with the preset balls there
                new MoveAction(fininshHPCollectPose),
                new DelayAction(550),
                //new GeneralAction(() -> doIntakePulse = true),
//                new GeneralAction(() -> shouldFireUnsortedBalls = true),
                new MoveAction(small_triangle_shoot),
//                new DelayAction(400),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1150),
                // end of hp collect and shooting


                // third row collecting and shooting
                new MoveAction(thirdRowCollectDone, BezierCurveTypes.ConstantHeading,thirdRowCollectDone.getHeading(), bezierHelper1),
                //new GeneralAction(() -> doIntakePulse = true),
//                new GeneralAction(() -> shouldFireUnsortedBalls = true),
                new MoveAction(small_triangle_shoot),
//                new DelayAction(400),
                new GeneralAction(fireUnsortedBalls),
                new GeneralAction(() -> limelight3A.pipelineSwitch(7)), // switch channel only once
                new DelayAction(1000),
                // finished third row shooting


                // 4th cycle, camera collecting
                new GeneralAction(scanForBallsAndPlanPath),
                // actual collect and firing
                new MoveAction(true),
                new DelayAction(300),
                new GeneralAction(() -> doIntakePulse = true),
//                new GeneralAction(() -> shouldFireUnsortedBalls = true),
                new MoveAction(small_triangle_shoot),
//                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished firing camera cycle



                // 5th cycle, camera collecting
                new GeneralAction(scanForBallsAndPlanPath),
                // actual collect and firing
                new MoveAction(true),
                new DelayAction(300),
                new GeneralAction(() -> doIntakePulse = true),
//                new GeneralAction(() -> shouldFireUnsortedBalls = true),
                new MoveAction(small_triangle_shoot),
//                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished firing camera cycle




                // 6th cycle, camera collecting
                new GeneralAction(scanForBallsAndPlanPath),
                // actual collect and firing
                new MoveAction(true),
                new DelayAction(300),
                new GeneralAction(() -> doIntakePulse = true),
//                new GeneralAction(() -> shouldFireUnsortedBalls = true),
                new MoveAction(small_triangle_shoot),
//                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished firing camera cycle



                // 7th cycle, camera collecting CUSTOM CYCLE
                new GeneralAction(scanForBallsAndPlanPath),
                // actual collect and firing
                new MoveAction(secondZoneCameraCollect,BezierCurveTypes.ConstantHeading,bezierHelper2.getHeading(), bezierHelper2),
                new DelayAction(300),
                new GeneralAction(() -> doIntakePulse = true),
//                new GeneralAction(() -> shouldFireUnsortedBalls = true),
                new MoveAction(small_triangle_shoot),
//                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished firing camera cycle

                
                // 8th cycle, camera collecting
                new GeneralAction(scanForBallsAndPlanPath),
                // actual collect and firing
                new MoveAction(true),
                new DelayAction(300),
                new GeneralAction(() -> doIntakePulse = true),
//                new GeneralAction(() -> shouldFireUnsortedBalls = true),
                new MoveAction(small_triangle_shoot),
//                new DelayAction(150),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished firing camera cycle



                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),
                new MoveAction(parkPose)
        );
    }
    public Runnable scanForBallsAndPlanPath = () -> {
        int cameraCase = (int) getBallNumber();
//        if(wentTooNumber2 >0) cameraCase = 1; not needed with new rolers
        //if(wentTooNumber2 >0) cameraCase = 1; // temporary untuill we get camera back
        switch (cameraCase){
            case 1: GlobalStorage.futureMoveActionTargetPose = fininshHPCollectPoseNEW; break;
            case 2: GlobalStorage.futureMoveActionTargetPose = secondZoneCameraCollect; wentTooNumber2++; break;
            //case 3: GlobalStorage.futureMoveActionTargetPose = thirdZoneCameraCollect; break;
            default: GlobalStorage.futureMoveActionTargetPose = fininshHPCollectPoseNEW; break;
        }
    };
    Runnable checkEmptyIntake = () -> {
        if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.NoBall || calculatedRightSensorDetectedBall == BallColorSet_Decode.NoBall){
            // go to collect again
            robot.executeNow(
                    new GeneralAction(scanForBallsAndPlanPath),
                    new MoveAction(true)
            );
        }

    };
    public boolean checkIfNoBall(){
        return calculatedLeftSensorDetectedBall == BallColorSet_Decode.NoBall || calculatedRightSensorDetectedBall == BallColorSet_Decode.NoBall;
    }






    Runnable fireSortedBalls = () -> {
        switch (camId) {
            case 23: // ppg
                firePPG();
                break;

            case 22: // pgp
                firePGP();
                break;

            case 21: // gpp
                fireGPP();
                break;
        }
    };
    //===== shooting sorted ======/
    public void firePPG(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // left right right
                        new StateAction("LeftGateServo", "OPEN"), // left left
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence( // right right left
                        new StateAction("RightGateServo", "OPEN"), // right right
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green isnt or is in intake
                robot.executeNow(new ActionSequence( // right left right
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(mainTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
        }
    }
    /*
     */
    public void firePGP(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // left right left
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(timerToCloseGate),
                        new StateAction("LeftGateServo", "CLOSED"),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(mainTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence(// right left right
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(mainTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green ball is in intake
                robot.executeNow(new ActionSequence( // right right left
                        new StateAction("RightGateServo", "OPEN"), // right right
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
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
                robot.executeNow(new ActionSequence( // right left right
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(mainTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence( //left right left
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(timerToCloseGate),
                        new StateAction("LeftGateServo", "CLOSED"),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(mainTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green ball is in intake, cant actually sort this airsort needed
                robot.executeNow(new ActionSequence( // it willl fire right right left and try air sort
                        new StateAction("RightGateServo", "OPEN"), // right right
                        new GeneralAction(turnAirSortOff),
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
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
//                            new DelayAction(timer1),
                new StateAction("LeftGateServo", "OPEN"),
                new DelayAction(timer2),
                new StateAction("RightGateServo", "OPEN"),//

                // close gates
                new DelayAction(600),
                new StateAction("RightGateServo", "CLOSED"),
                new StateAction("LeftGateServo", "CLOSED")
        ));
    };
    public void checkToFireUnsortedBalls(boolean shouldFire){
        if(calculateDistance(robot.getCurrentPose(),small_triangle_shoot,true) < 0.25 && shouldFire){
            shouldFireUnsortedBalls = false;
            robot.executeNow(new ActionSequence(
                    new DelayAction(80),
                    new StateAction("RightGateServo", "OPEN"),
                    new DelayAction(timerToCloseGate),
                    new StateAction("RightGateServo", "CLOSED"),
//                            new DelayAction(timer1),
                    new StateAction("LeftGateServo", "OPEN"),
                    new DelayAction(timer2),
                    new StateAction("RightGateServo", "OPEN"),//

                    // close gates
                    new DelayAction(600),
                    new StateAction("RightGateServo", "CLOSED"),
                    new StateAction("LeftGateServo", "CLOSED")
            ));
        }
    }
    public Runnable makeFireUnsortedBalls (double timeToWait){
        return  () -> {
            robot.executeNow(new ActionSequence(
                    new DelayAction(timeToWait),
                    new StateAction("RightGateServo", "OPEN"),
                    new DelayAction(timerToCloseGate),
                    new StateAction("RightGateServo", "CLOSED"),
//                            new DelayAction(timer1),
                    new StateAction("LeftGateServo", "OPEN"),
                    new DelayAction(timer2),
                    new StateAction("RightGateServo", "OPEN"),//

                    // close gates
                    new DelayAction(600),
                    new StateAction("RightGateServo", "CLOSED"),
                    new StateAction("LeftGateServo", "CLOSED")
            ));
        };
    }






    Runnable turnAirSortOff = () -> {
        robot.executeNow(new ActionSequence(
                new DelayAction(timeToTurnAirSortOff),
                new GeneralAction(() -> shouldToAirSort = false)
        ));
    };

    Runnable turnStuffOff = () -> {
        shouldFire = false;
        doIntakePulse = false;
        shouldRemoveBalls = false;

        robot.getMotorComponent("TurretSpinMotor")
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setTarget(0);
        robot.executeNow(new StateAction("TurretAngle", "DEFAULT"));
        robot.getTurretComponent("TurretRotateMotor").setTarget(0);
    };
    TurretComponent turret;
    public void firingTurret(boolean shouldFire) {
        turret = robot.getTurretComponent("TurretRotateMotor");
        double distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetXForFarAuto, cfg.targetYForFarAuto);
        double rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetXForFarAuto, cfg.targetYForFarAuto);
        //rotationToWallOdometry += cfg.autoZoneAdderFar;
        if(shouldFire){
            turret.setBallTimeInAir(ballInAirTime);
            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry) * vMultiplier/* + cfg.autoVelAdder*/;
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                    .setTarget(targetVelocity);


            double turretAngleVal = distanceToAngleFunction(distanceToWallOdometry);
            robot.getServoComponent("TurretAngle")
                    .setTarget(turretAngleVal);

            if(shouldHoldTurretForCameraScan)
                rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetForCameraX, cfg.targetForCameraY);;

            if(rotationToWallOdometry < 0) rotationToWallOdometry += 360;
            turret.setTarget(rotationToWallOdometry)
            ;
        }
    }
    public void cameraStuffUpdates(double targetX, double targetY){
        double camAngle = - calculateCameraAngle(targetX,targetY,robot.getCurrentPose(),camOffsetX,0);
        double cameraAngle = convertCamAngleToServoValue(camAngle);
        cameraAngle = clamp(cameraAngle,0,360); // de notat ca are range de 310 grade defapt

        robot.getServoComponent("CameraRotateServo")
                .setTarget((eval(cameraAngleOverite) ? cameraAngleOverite : cameraAngle));
    }
    protected void handleColors() {
        leftSensorColors = colorSensorLeft.getNormalizedColors();
        rightSensorColors = colorSensorRight.getNormalizedColors();

        Color.colorToHSV(leftSensorColors.toColor(), hsvLeftSensorColors);
        Color.colorToHSV(rightSensorColors.toColor(), hsvRightSensorColors);

        actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors,true);
        actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if(shouldResetRightSensorBall && resetLeftBallColorTimer.milliseconds() > 450){
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

        hasBallInLeftChamber = (actualRightSensorDetectedBall != BallColorSet_Decode.NoBall);
        hasBallInRightChamber = (actualRightSensorDetectedBall != BallColorSet_Decode.NoBall);

        /// distance sesnsor stuff

        // Read sensor voltage (0.0–3.3V)
        double volts = laserAnalog.getVoltage();
        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
        hasBallInIntake = distanceMM < ballInIntakeThreshold;


        RobotController.telemetry.addData("LEFT_RED", (double)leftSensorColors.red * 10000.0 * leftSensorColorMultiplier);
        RobotController.telemetry.addData("LEFT_BLUE", (double)leftSensorColors.blue * 10000.0 * leftSensorColorMultiplier);
        RobotController.telemetry.addData("LEFT_GREEN", (double)leftSensorColors.green * 10000.0 * leftSensorColorMultiplier);

        RobotController.telemetry.addData("RIGHT_RED", (double)rightSensorColors.red * 10000.0);
        RobotController.telemetry.addData("RIGHT_BLUE", (double)rightSensorColors.blue * 10000.0);
        RobotController.telemetry.addData("RIGHT_GREEN", (double)rightSensorColors.green * 10000.0);

        RobotController.telemetry.addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
        RobotController.telemetry.addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
        RobotController.telemetry.addData("Has ball in intake", hasBallInIntake);

    }
    public static int lastGateState = 0;
    protected void pulseIntake(boolean shouldPulseIntake){
        if(shouldPulseIntake){
            doIntakePulse = false;
            robot.executeNow(new ActionSequence(
                    new DelayAction(50),
                    new StateAction("IntakeMotor","OFF"),
                    new DelayAction(45),
                    new StateAction("IntakeMotor","FULL")
            ));
        }
    }
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Blue);
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
        if(camId < 21 || camId > 23) camId = 23;
    }
    public void convertPoses() {
        starter = convertPose(starter);
        parkPose = convertPose(parkPose);
        small_triangle_shoot = convertPose(small_triangle_shoot);
        fininshHPCollectPose = convertPose(fininshHPCollectPose);
        fininshHPCollectPoseNEW = convertPose(fininshHPCollectPoseNEW);
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
        bezierHelper2 = convertPose(bezierHelper2);
    }
    public Pose convertPose(Pose pose) {
        return pose;
    }
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose();
        return globalRobotPose;
    }
    public double getBallNumber(){
//        if(shouldSwitchChannel){
//            limelight3A.pipelineSwitch(1);
//            shouldSwitchChannel = false;
//        }
        LLResult llResult = limelight3A.getLatestResult();
        limelight3A.captureSnapshot("HpSnap");

        if (llResult != null) {
            double[] pythonData = llResult.getPythonOutput();
            if (pythonData.length > 0) {
                double firstValue = pythonData[0];
                RobotController.telemetry.addData("Python Val 1", firstValue);
                return firstValue;
            }
        }
        return 0;
    }
    public void EmergencyOverrideAtTheEnd(){
        robot.clearMainQueue();
        robot.executeNow(new ActionSequence(
                new HoldAction(500),
                new GeneralAction(turnStuffOff)
        ));
    }
}