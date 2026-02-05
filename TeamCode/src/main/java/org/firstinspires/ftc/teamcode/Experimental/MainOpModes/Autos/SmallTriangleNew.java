package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.farZoneCameraAdder;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.rotationForInitSmallTriangle;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateDistanceToWallInMeters;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateHeadingAdjustment;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

import java.io.IOException;
import java.util.List;

@Config
@Autonomous(name = "Small Triangle Auto BLUE", group = "AAA")
public class SmallTriangleNew extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private Limelight3A limelight3A;
    public static MainConfig cfg;
    private boolean shouldFire;
    public static boolean isMoving;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime resetLeftBallColorTimer = new ElapsedTime();
    public static boolean shouldMoveIntakeServo = false;
    public static boolean doIntakePulse = false;

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
    public static int camId =23;
    private Pose starter = pose( 0.0, 6.12, 90);
    private Pose small_triangle_shoot = pose(1, 3, 90);
    private Pose parkPose = pose(10, 15, 90);
    private Pose fininshHPCollectPose = pose(0,46,90);
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
                HandleColors();
                if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(fininshHPCollectPose) && !ComplexFollower.done()) ComplexFollower.interrupt();
                firingTurret(shouldFire);
                intakeChecks(shouldMoveIntakeServo);
                pulseIntake(doIntakePulse);
            }
        };
        makeConfig();
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder();
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        convertPoses();
        shouldFire = false; lastGateState = 0; resetLeftBallColorTimer.reset();
        shouldRemoveBalls = false;shouldResetRightSensorBall = false; doIntakePulse = false;

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
    private void makeAuto() {
        robot.addToQueue(
                new GeneralAction(prepQueueToFireSortedBall),
                new GeneralAction(turnOnIntakeServo),
                new GeneralAction(() -> shouldFire = true),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(3500),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new StateAction("IntakeMotor","OFF"),

                new StateAction("IntakeMotor","FULL"),

                new MoveAction(fininshHPCollectPose),
                new DelayAction(300),
                new GeneralAction(prepQueueToFireSortedBall),
                new GeneralAction(() -> {
                    shouldFire = true;
                    doIntakePulse = true;
                }),
                new MoveAction(small_triangle_shoot),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new StateAction("IntakeMotor","OFF"),

                new GeneralAction(turnOnIntakeServo),
                new StateAction("IntakeMotor","FULL"),

                new MoveAction(fininshHPCollectPose),
                new GeneralAction(prepQueueToFireSortedBall),
                new GeneralAction(() -> {
                    shouldFire = true;
                    doIntakePulse = true;

                }),
                new MoveAction(small_triangle_shoot),
                new GeneralAction(() -> doIntakePulse = true),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),




                new MoveAction(fininshHPCollectPose),
                new GeneralAction(prepQueueToFireSortedBall),
                new GeneralAction(() -> {
                    shouldFire = true;
                    doIntakePulse = true;
                }),
                new MoveAction(small_triangle_shoot),
                new GeneralAction(() -> doIntakePulse = true),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),





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
    Runnable turnStuffOff = () -> {
        shouldFire = false;
        shouldMoveIntakeServo = false;
        doIntakePulse = false;
        shouldRemoveBalls = false;

        robot.getMotorComponent("TurretSpinMotor")
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setTarget(0);
        robot.executeNow(new StateAction("TurretAngle", "DEFAULT"));
        robot.getMotorComponent("TurretRotateMotor").setTarget(0);
    };
    public void firingTurret(boolean shouldFire) {
        double distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetX, cfg.targetY);
        double rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetXLeftPanel, cfg.targetYLeftPanel);
        rotationToWallOdometry += cfg.autoZoneAdderFar;
        if(rotationToWallOdometry < -30) rotationToWallOdometry += 360;

        if(shouldFire){

            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry) + cfg.autoVelAdder;
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Velocity)
                    .setTarget(targetVelocity);


            double turretAngleVal = angle;
            turretAngleVal = clamp(turretAngleVal,262,324);
            robot.getServoComponent("TurretAngle")
                    .setTarget(turretAngleVal);


            robot.getMotorComponent("TurretRotateMotor")
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
    protected void intakeChecks(boolean shouldCheck){
        int gateState = 0;
        if(shouldCheck) {
            if (!hasBallInRightChamber) gateState = 1;
            else if (!hasBallInLeftChamber) gateState = -1;
            else gateState = 0;
        }
        else gateState = lastGateState;
        lastGateState = gateState;
    }
    Runnable turnOnIntakeServo = () -> {
        shouldMoveIntakeServo = true;
    };
    Runnable turnOffIntakeServo = () -> {
        shouldMoveIntakeServo = false;
    };
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Blue);
    }
    public void useCamera() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
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
}