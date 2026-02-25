package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftSensorColorMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.pose;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateDistanceToWallInMeters;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateHeadingAdjustment;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer3;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer4;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timerToCloseGate;

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
import java.util.List;

@Config
@Autonomous(name = "Big Triangle Auto BLUE", group = "AAA")
public class BigTriangleArtefactAuto extends OpMode {
    public RobotController robot;
    private AutoRecorder recorder;
    private Limelight3A limelight3A;
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
    /// --------------------------------------------------------
    private Pose closeStarter = pose(122, 26.3, 180); // would also be around 1.4x
    private Pose small_triangle_shoot = pose(1.5, 8, 90);
    private Pose parkPose = pose(1, 22, 90);
    private Pose fininshHPCollectPose = pose(1.8,44,90); // hp collect
    private Pose secondZoneCameraCollect = pose(17.8, 44, 90);
    private Pose thirdZoneCameraCollect = pose(33.96, 44, 90);
    private Pose thirdRowCollectDone = pose(30, 42, 90); // third row done
    private Pose secondRowCollectDone = pose(51.7, 36, 90);
    private Pose firstRowCollectDone = pose(77.5, 36, 90);
    private Pose gateCollect = pose(52.691, 41.142, 64.358);
    private Pose tipBigTriangleShooting = pose(67, -3, 180);
    private Pose tipBigTriangleShootingTurned90Deg = pose(67, -3, 90);
    private Pose middleBigTriangleShooting = pose(88.2, -3, 180);
    private Pose middleBigTriangleShootingTurned90Deg = pose(88.2, -3, 90);
    private Pose parkedBigTriangleShooting = pose(104.2, 0, 180);
    private Pose gateOpen = pose(60.1, 36.7, 90);
    private Pose bezierHelper1 = pose(32, 4, 90);
    public static double distanceToWallOdometry;
    public static double rotationToWallOdometry;
    public static int camId = 23;

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
            }

            private void controls() { // this will happen in a loop
                isMoving = ComplexFollower.instance().isBusy();
                if(shouldUseColorSensors) HandleColors();
                firingTurret(shouldFire);
                if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(gateCollect) && !ComplexFollower.done()) ComplexFollower.interrupt();

                distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetXAutoClose, cfg.targetYAutoClose);
                rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetXAutoClose, cfg.targetYAutoClose);
                if(rotationToWallOdometry < 0) rotationToWallOdometry += 360;

                if (startAuto) {
                    startAuto = false;
                    makeLeverAuto();
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
        collectNumber = 0;
        movingTimer.reset();
        convertPoses();
    }

    @Override
    public void init_loop() {
        robot.init_loop();
        robot.getTurretComponent("TurretRotateMotor").setTarget(cfg.rotationForInitClsoeZone);
        useCamera();
        RobotController.telemetry.addData("id",camId);
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
                new DelayAction(1400),
                /// finished preload and path



                /// second row
                new GeneralAction(() -> collectNumber++),
                //new MoveAction(second_row_ready),
                new MoveAction(secondRowCollectDone,true,BezierCurveTypes.TangentHeading,0),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1400),
                /// end of second row firing


                /// collecting lever pose not implemented yet so its just the second row logic again
                new GeneralAction(() -> collectNumber++),
                new MoveAction(secondRowCollectDone),
                new MoveAction(tipBigTriangleShootingTurned90Deg),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1400),
                ///finished gate collect


                /// collecting the first row
                new GeneralAction(() -> collectNumber++),
                new MoveAction(firstRowCollectDone),
                new MoveAction(tipBigTriangleShooting),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new DelayAction(150),
                new GeneralAction(prepToFireSortedBall),
                new GeneralAction(fireSortedBall),
                new DelayAction(450),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(400),
                new GeneralAction(() -> shouldUseColorSensors = false),
                /// end of first row firing


                /// beginning of third row collect
                new GeneralAction(() -> collectNumber++),
                new MoveAction(thirdRowCollectDone,true,BezierCurveTypes.TangentHeading,0),
                new MoveAction(parkedBigTriangleShooting,false,BezierCurveTypes.ReverseTangentHeading,0),
                new GeneralAction(() -> shouldUseColorSensors = true),
                new DelayAction(200),
                new GeneralAction(prepToFireSortedBall),
                new GeneralAction(fireSortedBall),
                new DelayAction(450),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(() -> shouldUseColorSensors = false),
                ///end of third row firing

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
            if(shouldBoostOnTheGoVelocityLogic) rotationToWallOdometry += rotationOnTheGo;
            robot.getTurretComponent("TurretRotateMotor")
                    .setTarget(rotationToWallOdometry)
            ;
            return;
        }
    }
    Runnable prepToFireSortedBall = () -> {
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
            robot.executeNow(new ActionSequence(
                    new StateAction("RightGateServo", "OPEN"),
                    new DelayAction(300),
                    new GeneralAction(() -> calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall),
                    new StateAction("RightGateServo", "CLOSED")
            ));
        }
        else if (ballToFire == calculatedLeftSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
            robot.executeNow(new ActionSequence(
                    new StateAction("LeftGateServo", "OPEN"),
                    new DelayAction(300),
                    new GeneralAction(() -> calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall),
                    new StateAction("LeftGateServo", "CLOSED")
            ));
        }
        else // if cant sort
        {
            if(calculatedLeftSensorDetectedBall != BallColorSet_Decode.NoBall){
                robot.executeNow(new ActionSequence(
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(300),
                        new GeneralAction(() -> calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
            }
            else if(calculatedRightSensorDetectedBall != BallColorSet_Decode.NoBall){
                robot.executeNow(new ActionSequence(
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(300),
                        new GeneralAction(() -> calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new StateAction("RightGateServo", "CLOSED")
                ));
            }
            else{ // IF REAAALLLYYY no ball
                robot.executeNow(new ActionSequence(
                        new StateAction("RightGateServo", "OPEN"),
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(500),
                        new GeneralAction(() -> calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new GeneralAction(() -> calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
            }

        }
        ballToFire = BallColorSet_Decode.NoBall;
    };
    Runnable fireUnsortedBalls = () -> {
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
    };
    Runnable turnStuffOff = () -> {
        shouldFire = false;

        robot.getMotorComponent("TurretSpinMotor")
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setTarget(0);
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT")); // go to default position
            robot.getTurretComponent("TurretRotateMotor").setTarget(0);
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
        tipBigTriangleShooting = convertPose(tipBigTriangleShooting);
        middleBigTriangleShooting = convertPose(middleBigTriangleShooting);
        tipBigTriangleShootingTurned90Deg = convertPose(tipBigTriangleShootingTurned90Deg);
        middleBigTriangleShootingTurned90Deg = convertPose(middleBigTriangleShootingTurned90Deg);
        parkedBigTriangleShooting = convertPose(parkedBigTriangleShooting);
        gateOpen = convertPose(gateOpen);
        bezierHelper1 = convertPose(bezierHelper1);
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
    }
    public Pose convertPose(Pose pose){
        return pose;
    }
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose();
        return globalRobotPose;
    }
}