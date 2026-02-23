package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.farZoneCameraAdder;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.rotationForInitSmallTriangle;
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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

import java.io.IOException;
import java.util.List;

@Config
@Autonomous(name = "Small Triangle Auto BLUE", group = "AAA")
public class SmallTriangleNew extends OpMode {
    private RobotController robot;
    //private AutoRecorder recorder;
    private Limelight3A limelight3A;
    public static MainConfig cfg;
    private boolean shouldFire;
    public static boolean isMoving;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime resetLeftBallColorTimer = new ElapsedTime();
    public static boolean doIntakePulse = false;
    public static boolean shouldHoldTurretForCameraScan = false;

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
    BallColorQueue ballColorQueue = new BallColorQueue();
    public static double velocity = 1480;
    public static double angle = 266;
    public static double rotationNeededForCameraScan = 15; // about 15 degrees to scan with limelight
    public static int camId =23;
    private Pose starter = pose( 0.0, 13.6, 90); // would also be around 1.4x
    private Pose small_triangle_shoot = pose(1, 10, 90);
    private Pose parkPose = pose(10, 15, 90);
    private Pose fininshHPCollectPose = pose(0,46,90);
    private Pose firstZoneCameraCollect = pose(0, 45.8, 90);
    private Pose secondZoneCameraCollect = pose(14, 45.5, 90);
    private Pose thirdZoneCameraCollect = pose(33, 45.1, 90);
    private Pose bezierHelper1 = pose(6.3, 20, 90);
    private Pose bezierHelper2 = pose(39, 10, 90);
    private Pose finalThirdRow = pose(29.3, 38.9, 90);

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
                //HandleColors();
                if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(fininshHPCollectPose) && !ComplexFollower.done()) ComplexFollower.interrupt();
                firingTurret(shouldFire);
                pulseIntake(doIntakePulse);
            }
        };
        makeConfig();
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        //recorder = new AutoRecorder();
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        convertPoses();
        shouldFire = false; lastGateState = 0; resetLeftBallColorTimer.reset();
        shouldRemoveBalls = false;shouldResetRightSensorBall = false; doIntakePulse = false;

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
    }

    @Override
    public void init_loop() {
        robot.init_loop();
        robot.getMotorComponent("TurretRotateMotor").setTarget(cfg.rotationForInitSmallTriangle);
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
                new DelayAction(3000), // revving up outtake
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // end of preload

                //first hp collect with the preset balls there
                new MoveAction(fininshHPCollectPose),
                new DelayAction(300),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(400),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // end of hp collect and shooting


                // third row collecting and shooting
                new MoveAction(finalThirdRow, BezierCurveTypes.TangentHeading,0, bezierHelper1, bezierHelper2),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot, BezierCurveTypes.ReverseTangentHeading,0, bezierHelper2, bezierHelper1),
                new DelayAction(400),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished third row shooting


                // 4th cycle, camera collecting
                new GeneralAction(() -> shouldHoldTurretForCameraScan = true),
                new DelayAction(800),
                new GeneralAction(scanForBallsAndPlanPath),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = false),
                // actual collect and firing
                new MoveAction(true),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(500),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished firing camera cycle


                // 5th cycle, camera collecting
                new GeneralAction(() -> shouldHoldTurretForCameraScan = true),
                new DelayAction(800),
                new GeneralAction(scanForBallsAndPlanPath),
                new GeneralAction(() -> shouldHoldTurretForCameraScan = false),
                // actual collect and firing
                new MoveAction(true),
                new GeneralAction(() -> doIntakePulse = true),
                new MoveAction(small_triangle_shoot),
                new DelayAction(400),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1000),
                // finished firing camera cycle this was last cycle


                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),
                new MoveAction(parkPose)
        );
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
        else
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
            else{
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
        doIntakePulse = false;
        shouldRemoveBalls = false;

        robot.getMotorComponent("TurretSpinMotor")
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setTarget(0);
        robot.executeNow(new StateAction("TurretAngle", "DEFAULT"));
        robot.getTurretComponent("TurretRotateMotor").setTarget(0);
    };
    public void firingTurret(boolean shouldFire) {
        double distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetX, cfg.targetY);
        double rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetXLeftPanel, cfg.targetYLeftPanel);
        //rotationToWallOdometry += cfg.autoZoneAdderFar;
        if(rotationToWallOdometry < 0) rotationToWallOdometry += 360;

        if(shouldFire){

            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry)/* + cfg.autoVelAdder*/;
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                    .setTarget(targetVelocity);


            double turretAngleVal = distanceToAngleFunction(distanceToWallOdometry);
            robot.getServoComponent("TurretAngle")
                    .setTarget(turretAngleVal);

            if(shouldHoldTurretForCameraScan) rotationToWallOdometry = rotationNeededForCameraScan;
            robot.getTurretComponent("TurretRotateMotor")
                    .setTarget(rotationToWallOdometry)
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
    public static int lastGateState = 0;
    protected void pulseIntake(boolean shouldPulseIntake){
        if(shouldPulseIntake){
            doIntakePulse = false;
            robot.executeNow(new ActionSequence(

                    new StateAction("IntakeMotor","FULL_REVERSE"),
                    new DelayAction(50),
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
    }
    public Pose convertPose(Pose pose) {
        return pose;
    }
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose();
        return globalRobotPose;
    }
    public double getBallNumber(){
        limelight3A.pipelineSwitch(1);
        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null) {
            double[] pythonData = llResult.getPythonOutput();
            if (pythonData.length > 0) {
                double firstValue = pythonData[0];

                return firstValue;
//                telemetry.addData("Python Val 1", firstValue);

            }
        }
        return 0;
    }
    public Runnable scanForBallsAndPlanPath = () -> {
        int cameraCase = (int) getBallNumber();
        switch (cameraCase){
            case 1: GlobalStorage.futureMoveActionTargetPose = firstZoneCameraCollect; break;
            case 2: GlobalStorage.futureMoveActionTargetPose = secondZoneCameraCollect; break;
            case 3: GlobalStorage.futureMoveActionTargetPose = thirdZoneCameraCollect; break;
            default: GlobalStorage.futureMoveActionTargetPose = fininshHPCollectPose; break;
        }
    };
}