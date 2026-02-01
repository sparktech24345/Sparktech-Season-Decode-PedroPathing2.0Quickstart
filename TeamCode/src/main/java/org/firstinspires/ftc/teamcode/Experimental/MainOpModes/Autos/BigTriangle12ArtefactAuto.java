package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftSensorColorMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.normalAngle;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.pose;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.angleDecreaseValue;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateDistanceToWallInMeters;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateHeadingAdjustment;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.velocityDeltaCompensation;

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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

import java.io.IOException;
import java.util.List;

@Config
@Autonomous(name = "Big Triangle Auto BLUE", group = "AAA")
public class BigTriangle12ArtefactAuto extends OpMode {
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
    public static boolean shouldPullFromQueue = false;
    BallColorQueue ballColorQueue = new BallColorQueue();
    public static boolean shouldFire = false;
    public static boolean shouldMoveIntakeServo = false;
    /// --------------------------------------------------------
    public  Pose starter = pose(0.0, 0.0, 0.0); // Default Start Position (p0)
    public  Pose first_row_ready = pose(35, -2, 120); // Pose4: collect first row right
    public  Pose first_row_intermediate = pose(31, 13, 90); // Pose4: collect first row right
    public  Pose first_row_done = pose(31, 46.5, 90); // Pose5: collect first row left
    public  Pose second_row_ready = pose(53, 12, 90); // Pose7: collect second row right
    public  Pose second_row_done = pose(52, 42, 90); // Pose8: colect second row left
    public  Pose leverPoseSecondRow = pose(65, 39.5, 90); // Pose8: colect second row left
    public  Pose leverPoseThirdRow = pose(65.5, 39, 90); // Pose8: colect second row left
    public  Pose big_triangle_shoot_third_collect = pose(75, 0, 90); // Pose9: shooting big triangle pose
    public  Pose big_triangle_shoot_second_collect = pose(68, 4, 90); // Pose9: shooting big triangle pose
    public  Pose big_triangle_shoot_third_collect_with_park = pose(100, 0, 90); // Pose9: shooting big triangle pose
    public  Pose big_triangle_shoot_third_collect_with_park_180 = pose(100, 0, 180); // Pose9: shooting big triangle pose
    public  Pose third_row_ready = pose(75, 0, 90); // Pose10: collect third row right
    public  Pose third_row_done = pose(75, 30, 90); // Pose11: collect third row left
    public Pose classifier_starter = pose(120, 27, 90);

    //public  Pose hp_ready = pose(30,25,130);
    public Pose hp_collect_pose = pose(4,47,180);
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
                HandleColors();
                intakeChecks(shouldMoveIntakeServo);
                firingTurret(shouldFire);
                if (ComplexFollower.followingForMS() > 1000 && ComplexFollower.getTarget().equals(leverPoseSecondRow) && !ComplexFollower.done()) ComplexFollower.interrupt();
                if (ComplexFollower.followingForMS() > 1000 && ComplexFollower.getTarget().equals(leverPoseThirdRow) && !ComplexFollower.done()) ComplexFollower.interrupt();
                //bigIffMethod();

                distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetXAutoClose, cfg.targetYAutoClose);
                rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetXAutoClose, cfg.targetYAutoClose);
                if(rotationToWallOdometry < -30) rotationToWallOdometry += 360;

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
        shouldFire = false; shouldMoveIntakeServo = false; lastGateState = 1;
        hadBallInRightChamberInPast = false; hadBallInLeftChamberInPast = false;
        collectNumber = 0;
        movingTimer.reset();
        convertPoses();
        //teamSensitiveStuff();
    }

    @Override
    public void init_loop() {
        robot.init_loop();
        robot.getMotorComponent("TurretRotateMotor").setTarget(cfg.rotationForInitClsoeZone);
        robot.executeNow(new StateAction("IntakeSorterServo", "REDIRECT_TO_RIGHT"));
        useCamera();
        RobotController.telemetry.addData("id",camId);
    }

    @Override
    public void start() {
        ComplexFollower.setPose(classifier_starter);
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
    private PathConstraints brutalConstraints = new PathConstraints( // copiate direct din exemplul Pedro, de verificat / corectat
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.5,
            10,
            0.8
    );
    public void makeSortedAuto(){
        robot.addToQueue(
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                //new StateAction("IntakeMotor","OFF"),
                //new GeneralAction(turnStuffOff),



                /// second row
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new MoveAction(second_row_ready),
                new MoveAction(second_row_done),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                /// end of second row firing



                /// beginning of first row collect
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new MoveAction(first_row_ready),
                new MoveAction(first_row_done),
                new StateAction("IntakeMotor","OFF"),
                new DelayAction(200),
                ///firing the 12th ball firing
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect), ///TODO might change to park
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(400),
                /// end of first row firing




                /// collecting the third row

                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                //new MoveAction(third_row_ready),
                new MoveAction(third_row_done),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new MoveAction(big_triangle_shoot_third_collect_with_park),
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                /// end of third row firing



                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff)
        );
    }


    /// =============================== Lever Auto ============================


    public void makeLeverAuto(){
        robot.addToQueue(
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),

                /// collecting the first row

                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(third_row_ready),
                new MoveAction(third_row_done),
                new DelayAction(350),
                new MoveAction(leverPoseThirdRow),
                new DelayAction(500),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                /// end of first row firing



                /// second row
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(second_row_ready),
                new MoveAction(second_row_done),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                /// end of second row firing




                /// beginning of third row collect
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(first_row_ready),
                new MoveAction(first_row_done),
                ///firing the 12th ball firing
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect_with_park),
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(800),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(600),
                new GeneralAction(fireSortedBall),
                new DelayAction(800),
                /// end of 12 ball  row firing



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

            //double targetVelocity = FAR_TARGET_VELOCITY;
            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry);
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Velocity)
                    .setTarget(targetVelocity);


            // ----------------------- Angle Stuff -----------------------

            double turretAngleVal = normalAngle;
            turretAngleVal = clamp(turretAngleVal,262,324);

            //if(robot.getMotorComponent("TurretSpinMotor").getVelocity() + velocityDeltaCompensation <= targetVelocity) // +0.15 for safety
            //    turretAngleVal += angleDecreaseValue;
            robot.getServoComponent("TurretAngle")
                    .setTarget(turretAngleVal);

            // ----------------------- Rotation Stuff -----------------------

            robot.getMotorComponent("TurretRotateMotor")
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
    Runnable turnStuffOff = () -> {
        shouldFire = false;
        shouldMoveIntakeServo = false;

        robot.getMotorComponent("TurretSpinMotor")
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setTarget(0);
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT")); // go to default position
            robot.getMotorComponent("TurretRotateMotor").setTarget(0);
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
    protected void intakeChecks(boolean shouldCheck){
        int gateState = 0;
        if(/*!(collectNumber == 2 && camId == 23) || true*/ true){
            if(shouldCheck) {
                    if (!hasBallInRightChamber) gateState = -1; // first fill up left
                    else if (!hasBallInLeftChamber) gateState = 1; // then right
                    else gateState = 1; // then point middle cuz new sorting logic
            } // yet another logic
            else gateState = lastGateState;
            lastGateState = gateState;
        }
        else{
            if(shouldCheck) {
                if (!hasBallInLeftChamber) gateState = -1; // first fill up left
                else if (!hasBallInRightChamber) gateState = 1; // then right
                else gateState = -1; // then continue pointing to right for when you fire
                //else gateState = lastGateState;
            }
            else gateState = lastGateState;
            lastGateState = gateState;
        }
        // end of handicap


        switch (gateState) {
            case -1:
                robot.executeNow(new StateAction("IntakeSorterServo", "REDIRECT_TO_LEFT"));
                break;

            case 0:
                robot.executeNow(new StateAction("IntakeSorterServo", "BLOCK"));
                break;

            case 1:
                robot.executeNow(new StateAction("IntakeSorterServo", "REDIRECT_TO_RIGHT"));
                break;
        }
    }
    Runnable increaseCollectNumber = () -> {
        collectNumber++;
    };
    Runnable turnOnIntakeServo = () -> {
        shouldMoveIntakeServo = true;
    };
    Runnable turnOffIntakeServo = () -> {
        shouldMoveIntakeServo = false;
    };
//    public void teamSensitiveStuff(){
//        if(targetY < 0){
//            targetY = -targetY;
//        }
//        teamPipeline = 0;
//        currentTeamColor = TeamColor.Blue;
//    }
    public void makeConfig(){
    cfg = new MainConfig(MainConfig.Configs.Blue);
}
    public void convertPoses(){
        starter = convertPose(starter);

        first_row_ready = convertPose(first_row_ready);
        first_row_intermediate = convertPose(first_row_intermediate);
        first_row_done = convertPose(first_row_done);

        second_row_ready = convertPose(second_row_ready);
        //second_row_intermediate = convertPose(second_row_intermediate);
        //second_row_VERYintermediate = convertPose(second_row_VERYintermediate);
        second_row_done = convertPose(second_row_done);
        leverPoseSecondRow = convertPose(leverPoseSecondRow);
        leverPoseThirdRow = convertPose(leverPoseThirdRow);

        big_triangle_shoot_third_collect = convertPose(big_triangle_shoot_third_collect);
        big_triangle_shoot_third_collect_with_park = convertPose(big_triangle_shoot_third_collect_with_park);

        third_row_ready = convertPose(third_row_ready);
        //third_row_intermediate = convertPose(third_row_intermediate);
        //third_row_VERYintermediate = convertPose(third_row_VERYintermediate);
        third_row_done = convertPose(third_row_done);

        classifier_starter = convertPose(classifier_starter);
        //hp_ready = convertPose(hp_ready);
        hp_collect_pose = convertPose(hp_collect_pose);
        big_triangle_shoot_third_collect_with_park_180 = convertPose(big_triangle_shoot_third_collect_with_park_180);
        big_triangle_shoot_second_collect = convertPose(big_triangle_shoot_second_collect);

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
            //telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",, fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
        }
        if(camId < 21 || camId > 23) camId = 23;
    }
    public Pose convertPose(Pose pose){
        return pose;
    }
    public Pose passPose() {
        Pose tempPose = ComplexFollower.instance().getPose();
        globalRobotPose = tempPose;//new Pose(tempPose.getX(),tempPose.getY(),Math.toRadians(tempPose.getHeading())); //Math.toRadians
        return globalRobotPose;
    }
}