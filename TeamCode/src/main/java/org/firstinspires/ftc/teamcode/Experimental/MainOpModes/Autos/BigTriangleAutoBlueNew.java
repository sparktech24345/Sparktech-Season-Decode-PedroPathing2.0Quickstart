package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.pose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOPBlue.calculateDistanceToWallInMeters;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOPBlue.calculateHeadingAdjustment;

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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;
import java.util.List;

@Config
@Autonomous(name = "NEW BIG triangle BLUE", group = "AAA")
public class BigTriangleAutoBlueNew extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private Limelight3A limelight3A;
    private boolean startAuto = false;
    public static boolean isMoving;
    public static double fakeActionCounter = 0;
    public static double targetX = 127;
    public static double targetY = 48;
    public static double ballsLaunched = 0;
    public static double publicAngleConstantThingTemp = 20;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime movingTimer = new ElapsedTime();
    ElapsedTime isFiringTimer = new ElapsedTime();
    ElapsedTime shouldJiggle = new ElapsedTime();
    ElapsedTime shouldFinish = new ElapsedTime();
    private boolean had_balls = false;

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
    public static boolean hasBallInLeftChamber = false;
    public static boolean shouldRemoveBalls = false;
    public static boolean shouldPullFromQueue = false;
    BallColorQueue ballColorQueue = new BallColorQueue();
    public static double velocity = 780;
    public static double angle = 300;
    public static double rotation = 25;
    public static double rotationForInit = 145;
    public static boolean shouldFire = false;
    public static boolean shouldMoveIntakeServo = false;
    /// --------------------------------------------------------
    /*
private final Pose pose1 = new Pose(50.71300116110975, 15.411673718550073, Math.toRadians(90.11418277683327));
private final Pose pose2 = new Pose(50.31208669106792, 42.289693036417326, Math.toRadians(89.62438308126248));

private final Pose pose3 = new Pose(74.27231675996555, 15.024040402389888, Math.toRadians(89.96480653968509));
private final Pose pose4 = new Pose(73.85308693713091, 36.43503023883489, Math.toRadians(90.43687506417052));

private final Pose pose5 = new Pose(27.75532639871432, 15.633838082861715, Math.toRadians(89.57556088907523));
private final Pose pose6 = new Pose(27.75600403312623, 45.11518343227117, Math.toRadians(89.49693175135005));

private final Pose pose7 = new Pose(119.99206062376969, 26.829886849470963, Math.toRadians(90.3724663802973));*/
    private Pose starter = pose(0.0, 0.0, 0.0); // Default Start Position (p0)
    private Pose first_row_ready = pose(28, 15.5, 90); // Pose4: collect first row right
    private Pose first_row_done = pose(28, 45.2, 90); // Pose5: collect first row left
    private Pose second_row_ready = pose(53, 0, 90); // Pose7: collect second row right
    private Pose second_row_intermediate = pose(53, 15.5, 90); // Pose7: collect second row right
    private Pose second_row_VERYintermediate = pose(53, 20, 90); // Pose7: collect second row right
    private Pose second_row_done = pose(51, 44, 90); // Pose8: colect second row left

    private Pose big_triangle_shoot_third_collect = pose(75, 0, 90); // Pose9: shooting big triangle pose
    private Pose third_row_ready = pose(75, 0, 90); // Pose10: collect third row right
    private Pose third_row_intermediate = pose(75, 15, 90); // Pose10: collect third row right
    private Pose third_row_VERYintermediate = pose(75, 15, 90); // Pose10: collect third row right
    private Pose third_row_done = pose(75, 36, 90); // Pose11: collect third row left

    private Pose classifier_starter = pose(120, 27, 90);
    private Pose classifier_shooter = pose(117, 22, 90);
    private Pose classifier_park = pose(112, 13, 90);
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
                        }

            private void controls() { // this will happen in a loop
                isMoving = ComplexFollower.instance().isBusy();
                HandleColors();
                intakeChecks(shouldMoveIntakeServo);
                firingTurret(shouldFire);
                //bigIffMethod();

                distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), targetX, targetY);
                rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), targetX, targetY);
                if(rotationToWallOdometry < -30) rotationToWallOdometry += 360;

                if (startAuto) {
                    startAuto = false;
                    makeAuto();
                }
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder();
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        fakeActionCounter = 0;
        shouldFire = false; shouldMoveIntakeServo = false;
        movingTimer.reset();
        bIf1 = bIf2 = bIf3 = bIf4 = bIf5 = bIf6 = bIf7 = bIf8 = bIf9 = bIf10 =
                bIf11 = bIf12 = bIf13 = bIf14 = bIf15 = bIf16 = bIf17 = bIf18 = bIf19 = bIf20 =
                        bIf21 = bIf22 = bIf23 = bIf24 = bIf25 = bIf26 = bIf27 = bIf28 = bIf29 = bIf30 = true;
        convertPoses();
        teamSensitiveStuff();
    }

    @Override
    public void init_loop() {
        robot.init_loop();
        robot.getMotorComponent("TurretRotateMotor").setTarget(rotationForInit);
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
    private void makeAuto(){
        robot.addToQueue(
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),

                /// collecting the closest row after firing

                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(third_row_ready),
                new MoveAction(third_row_intermediate),
                //new DelayAction(100),
                //new MoveAction(third_row_VERYintermediate),
                new DelayAction(100),
                new MoveAction(third_row_done),
                new DelayAction(500),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new MoveAction(big_triangle_shoot_third_collect),


                /// fire the balls
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),


                /// second row
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(second_row_ready),
                new MoveAction(second_row_intermediate),
                new DelayAction(100),
                //new MoveAction(second_row_VERYintermediate),
               // new DelayAction(100),
                new MoveAction(second_row_done),
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
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),
                new GeneralAction(fireSortedBall),
                new DelayAction(1000),


                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),
                new MoveAction(classifier_park)
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

            double turretAngleVal = FAR_TARGET_ANGLE;
            turretAngleVal = clamp(turretAngleVal,262,324);
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
                        new StateAction("RightGateServo", "CLOSED")
                ));
            }
            else if (ballToFire == calculatedLeftSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
                robot.executeNow(new ActionSequence(
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(300),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
            }
            else // if cant sort
            {
                if(calculatedLeftSensorDetectedBall != BallColorSet_Decode.NoBall){
                    robot.executeNow(new ActionSequence(
                            new StateAction("LeftGateServo", "OPEN"),
                            new DelayAction(300),
                            new StateAction("LeftGateServo", "CLOSED")
                    ));
                }
                else if(calculatedRightSensorDetectedBall != BallColorSet_Decode.NoBall){
                    robot.executeNow(new ActionSequence(
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(300),
                            new StateAction("RightGateServo", "CLOSED")
                    ));
                }
                else{ // IF REAAALLLYYY no ball
                    robot.executeNow(new ActionSequence(
                            new StateAction("RightGateServo", "OPEN"),
                            new StateAction("LeftGateServo", "OPEN"),
                            new DelayAction(500),
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
    Runnable fakeAction = () -> ++fakeActionCounter;
    public static boolean bIf1, bIf2, bIf3, bIf4, bIf5, bIf6, bIf7, bIf8, bIf9, bIf10,
            bIf11, bIf12, bIf13, bIf14, bIf15, bIf16, bIf17, bIf18, bIf19, bIf20,
            bIf21, bIf22, bIf23, bIf24, bIf25, bIf26, bIf27, bIf28, bIf29, bIf30;

    public void bigIffMethod(){

        if(startAuto && bIf1){
            robot.executeNow(new MoveAction(classifier_shooter));
            robot.executeNow(new GeneralAction(prepToFireSortedBall));
            startAuto = false;
            bIf1 = false;
            movingTimer.reset();
        }
        if (bIf2 && !bIf1 && !isMoving && movingTimer.milliseconds() > 1000) {
            robot.addToQueue(new ActionSequence(
                    new StateAction("IntakeMotor","FULL"),
                    new DelayAction(2000),
                    new GeneralAction(fireSortedBall),
                    new DelayAction(1000),
                    new GeneralAction(fireSortedBall),
                    new DelayAction(1000),
                    new GeneralAction(fireSortedBall),
                    new DelayAction(1000),
                    new StateAction("IntakeMotor","OFF"),
                    new GeneralAction(turnStuffOff),
                    new GeneralAction(new Runnable() {
                        @Override
                        public void run() {
                            //bIf2 = false;
                        }
                    }),
                    new MoveAction(classifier_park)
                    //new StateAction("IntakeMotor","FULL")
            ));
        }
        if (bIf3 && !bIf2) {
            robot.executeNow(new GeneralAction(() -> {
                ComplexFollower.interrupt();
                robot.executeNow(new MoveAction(third_row_intermediate));
                movingTimer.reset();
                bIf3 = false;
            })
            );
        }
        if (bIf4 && !bIf3 && !isMoving) {
            robot.executeNow(
            new GeneralAction(() -> {
                ComplexFollower.interrupt();
                robot.executeNow(new MoveAction(third_row_ready));
                movingTimer.reset();
                bIf4 = false;
            })
            );
        }
        if (bIf5 && !bIf4 && !isMoving) {
            robot.executeNow(new GeneralAction(() -> {
                ComplexFollower.interrupt();
                robot.executeNow(new MoveAction(big_triangle_shoot_third_collect));
                movingTimer.reset();
                bIf5 = false;
            }));
        }
        if (bIf6 && !bIf5 && !isMoving) {
            robot.executeNow(new GeneralAction(() -> {
                ComplexFollower.interrupt();
                robot.executeNow(new MoveAction(third_row_done));
                movingTimer.reset();
                bIf6 = false;
            }));
        }
        if (bIf7 && !bIf6  && !isMoving) {
            robot.addToQueue(new ActionSequence(
                    new GeneralAction(prepToFireSortedBall),
                    new StateAction("IntakeMotor","FULL"),
                    new DelayAction(1000),
                    new GeneralAction(fireSortedBall),
                    new DelayAction(1000),
                    new GeneralAction(fireSortedBall),
                    new DelayAction(1000),
                    new GeneralAction(fireSortedBall),
                    new DelayAction(1000),
                    new StateAction("IntakeMotor","OFF"),
                    new GeneralAction(turnStuffOff),
                    new MoveAction(classifier_park)
            ));
            bIf7 = false;
        }
        if (bIf8 && !bIf7) {
            bIf8 = false;
        }
        if (bIf9 && !bIf8) {
            bIf9 = false;
        }
        if (bIf10 && !bIf9) {
            bIf10 = false;
        }
        if (bIf11 && !bIf10) {
            bIf11 = false;
        }
        if (bIf12 && !bIf11) {
            bIf12 = false;
        }
        if (bIf13 && !bIf12) {
            bIf13 = false;
        }
        if (bIf14 && !bIf13) {
            bIf14 = false;
        }
        if (bIf15 && !bIf14) {
            bIf15 = false;
        }
        if (bIf16 && !bIf15) {
            bIf16 = false;
        }
        if (bIf17 && !bIf16) {
            bIf17 = false;
        }
        if (bIf18 && !bIf17) {
            bIf18 = false;
        }
        if (bIf19 && !bIf18) {
            bIf19 = false;
        }
        if (bIf20 && !bIf19) {
            bIf20 = false;
        }
        if (bIf21 && !bIf20) {
            bIf21 = false;
        }
        if (bIf22 && !bIf21) {
            bIf22 = false;
        }
        if (bIf23 && !bIf22) {
            bIf23 = false;
        }
        if (bIf24 && !bIf23) {
            bIf24 = false;
        }
        if (bIf25 && !bIf24) {
            bIf25 = false;
        }
        if (bIf26 && !bIf25) {
            bIf26 = false;
        }
        if (bIf27 && !bIf26) {
            bIf27 = false;
        }
        if (bIf28 && !bIf27) {
            bIf28 = false;
        }
        if (bIf29 && !bIf28) {
            bIf29 = false;
        }
        if (bIf30 && !bIf29) {
            bIf30 = false;
        }

    }
    protected void HandleColors() {
        leftSensorColors = colorSensorLeft.getNormalizedColors();
        rightSensorColors = colorSensorRight.getNormalizedColors();

        Color.colorToHSV(leftSensorColors.toColor(), hsvLeftSensorColors);
        Color.colorToHSV(rightSensorColors.toColor(), hsvRightSensorColors);

        actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors);
        actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if (!shouldRemoveBalls && false) { // when not moving balls out of chambers they dont have permission to change to no ball
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

        RobotController.telemetry.addData("LEFT_RED",(double)leftSensorColors.red * 10000.0);
        RobotController.telemetry.addData("LEFT_BLUE",(double)leftSensorColors.blue * 10000.0);
        RobotController.telemetry.addData("LEFT_GREEN",(double)leftSensorColors.green * 10000.0);

        RobotController.telemetry.addData("RIGHT_RED",(double)rightSensorColors.red * 10000.0);
        RobotController.telemetry.addData("RIGHT_BLUE",(double)rightSensorColors.blue * 10000.0);
        RobotController.telemetry.addData("RIGHT_GREEN",(double)rightSensorColors.green * 10000.0);

        RobotController.telemetry.addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
        RobotController.telemetry.addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
    }
    protected void intakeChecks(boolean shouldCheck){
        int gateState = 0;
        if(shouldCheck){
            if (!hasBallInRightChamber) gateState = 1; // first fill up left
            else if (!hasBallInLeftChamber) gateState = -1; // then right
            else gateState = -  1; // then continue pointing to right for when you fire
        }
        else gateState = -1;
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
    Runnable turnOnIntakeServo = () -> {
        shouldMoveIntakeServo = true;
    };
    Runnable turnOffIntakeServo = () -> {
        shouldMoveIntakeServo = false;
    };
    public void teamSensitiveStuff(){
        if(targetY < 0){
            targetY = -targetY;
        }
        teamPipeline = 0;
        currentTeamColor = TeamColor.Blue;
    }
    public void convertPoses(){
        starter = convertPose(starter);
        first_row_ready = convertPose(first_row_ready);
        first_row_done = convertPose(first_row_done);
        second_row_ready = convertPose(second_row_ready);
        second_row_done = convertPose(second_row_done);
        big_triangle_shoot_third_collect = convertPose(big_triangle_shoot_third_collect);
        third_row_ready = convertPose(third_row_ready);
        third_row_intermediate = convertPose(third_row_intermediate);
        third_row_done = convertPose(third_row_done);
        classifier_starter = convertPose(classifier_starter);
        classifier_shooter = convertPose(classifier_shooter);
        classifier_park = convertPose(classifier_park);
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