package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;
import java.util.function.BooleanSupplier;

@Config
@Autonomous(name = "NEW BIG triangle BLUE", group = "Tests")
public class BigTriangleAutoBlueNew extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private boolean startAuto = false;
    public static boolean isMoving;
    public static double fakeActionCounter = 0;
    public static double targetX = 125;
    public static double targetY = 46;
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
    public static double velocity = 800;
    public static double angle = 310;
    public static double rotation = 25;
    /// --------------------------------------------------------
    private Pose starter = new Pose( 0.0, 0.0, 0.0); // Default Start Position (p0)
    private Pose small_triangle_shoot = new Pose(10, 0, 0); // Pose1: shooting position small triangle
    private Pose unstuckPose = new Pose(20, 0, 0); // Pose1: shooting position small triangle
    private Pose parkPose = new Pose(10, 15, 0);
    private Pose HP_collect = new Pose(38.6, -5.56, 0); // Pose3: HP collect
    private Pose first_row_ready = new Pose(15, 52, 0); // Pose4: collect first row right
    private Pose first_row_done = new Pose(30, 52, 0); // Pose5: collect first row left
    private Pose lever = new Pose(38.31, -62.65, 0); // Pose6: lever pose
    private Pose second_row_ready = new Pose(15, 77, 0); // Pose7: collect second row right
    private Pose second_row_done = new Pose(30, 77, 0); // Pose8: colect second row left
    private Pose big_triangle_shoot = new Pose(1, -90, 0); // Pose9: shooting big triangle pose
    private Pose big_triangle_offset = new Pose(1, -70, 0); // Pose9: shooting big triangle pose
    private Pose third_row_ready = new Pose(15, 100, 0); // Pose10: collect third row right
    private Pose third_row_done = new Pose(30, 100, 0); // Pose11: collect third row left
    private Pose third_row_firing = new Pose(30, 100, 0);
    private Pose classifier_starter = new Pose(130, 43.32, 90);
    private Pose classifier_shooter = new Pose(130, 36, 90);
    private Pose classifier_park = new Pose(130, 30, 90);

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

            private void controls() { // this will happen in a loop
                isMoving = robot.getFollowerInstance().instance().isBusy();
                HandleColors();
                if(startAuto){
                    startAuto = false;
                    makeAuto();
                }
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        fakeActionCounter = 0;
        movingTimer.reset();
    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
        robot.getFollowerInstance().setStartingPose(classifier_starter);
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
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        robot.executeNow(new MoveAction(classifier_shooter));
                        movingTimer.reset();
                    }
                }).setDoneCondition(supplyIsDoneMoving),
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

                /// collecting the closest row after firing

                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        robot.executeNow(new MoveAction(third_row_ready));
                        movingTimer.reset();
                    }
                }).setDoneCondition(supplyIsDoneMoving),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        robot.executeNow(new MoveAction(third_row_done));
                        movingTimer.reset();
                    }
                }).setDoneCondition(supplyIsDoneMoving),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        robot.executeNow(new MoveAction(third_row_firing));
                        movingTimer.reset();
                    }
                }).setDoneCondition(supplyIsDoneMoving),

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
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),
                new MoveAction(parkPose)
        );
    }
    /// Runnables
    Runnable prepToFireSortedBall = new Runnable() {
        @Override
        public void run() {
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Velocity)
                    .setTarget(velocity);
            // ----------------------- Angle Stuff -----------------------
            robot.getServoComponent("TurretAngle")
                    .setTarget(angle);
            // ----------------------- Rotation Stuff -----------------------
            robot.getMotorComponent("TurretRotateMotor")
                    .setTarget(rotation)
            ;
            ballColorQueue.clearQueue();
            int motifID=1; /// TODO DETECTION OF MOTIF
            switch (motifID){
                case 1:
                    ballColorQueue.add(BallColorSet_Decode.Purple);
                    ballColorQueue.add(BallColorSet_Decode.Purple);
                    ballColorQueue.add(BallColorSet_Decode.Green);
                    break;

                case 2:
                    ballColorQueue.add(BallColorSet_Decode.Purple);
                    ballColorQueue.add(BallColorSet_Decode.Green);
                    ballColorQueue.add(BallColorSet_Decode.Purple);
                    break;

                case 3:
                    ballColorQueue.add(BallColorSet_Decode.Green);
                    ballColorQueue.add(BallColorSet_Decode.Purple);
                    ballColorQueue.add(BallColorSet_Decode.Purple);
                    break;
            }}
    };

    Runnable fireSortedBall = new Runnable() {
        @Override
        public void run() {
            ballToFire = ballColorQueue.pull();

            if(ballToFire == calculatedRightSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall){
                robot.executeNow(new ActionSequence(
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(300),
                        new StateAction("RightGateServo", "CLOSED")
                ));
            }
            else if(ballToFire == calculatedLeftSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall){
                robot.executeNow(new ActionSequence(
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(300),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
            }
            else // if cant sort
            {
                robot.executeNow(new ActionSequence(
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(300),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
            }
            ballToFire = BallColorSet_Decode.NoBall;
        }
    };

    Runnable turnStuffOff = new Runnable() {
        @Override
        public void run() {
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Power)
                    .setTarget(0);
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT")); // go to default position
            robot.getMotorComponent("TurretRotateMotor").setTarget(0);
        }
    };

    Runnable fakeAction = new Runnable() {
        @Override
        public void run() {
            fakeActionCounter++;
        }
    };
    BooleanSupplier supplyIsDoneMoving = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return (!isMoving && movingTimer.milliseconds() > 20); // if not moving and not just began moving
        }
    };










    private void AutoPark() {
//        robot.addToQueue(new MoveAction(false, parkPose));
//        canSequence4 = false;
//        robot.addToQueue(new StateAction("IntakeMotor", "OFF"));
//        robot.getServoComponent("TurretRotateServo")
//                .setOverrideTarget_bool(true)
//                .setOverrideTargetPos(normalizeTurretRotationForServo(0));
//
//        robot.getMotorComponent("TurretSpinMotor")
//                .setOverrideCondition(true)
//                .setPowerOverride(0);
//
    }
    protected void HandleColors() {
        leftSensorColors = colorSensorLeft.getNormalizedColors();
        rightSensorColors = colorSensorRight.getNormalizedColors();

        Color.colorToHSV(leftSensorColors.toColor(), hsvLeftSensorColors);
        Color.colorToHSV(rightSensorColors.toColor(), hsvRightSensorColors);

        actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors);
        actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if(!shouldRemoveBalls){ // when not moving balls out of chambers they dont have permission to change to no ball
            if(actualLeftSensorDetectedBall != BallColorSet_Decode.NoBall)
                calculatedLeftSensorDetectedBall = actualLeftSensorDetectedBall;

            if(actualRightSensorDetectedBall != BallColorSet_Decode.NoBall)
                calculatedRightSensorDetectedBall = actualRightSensorDetectedBall;
        }
        else{
            calculatedLeftSensorDetectedBall = actualLeftSensorDetectedBall;
            calculatedRightSensorDetectedBall = actualRightSensorDetectedBall;
        }

        if(actualLeftSensorDetectedBall == null) actualLeftSensorDetectedBall = BallColorSet_Decode.NoBall;
        if(actualRightSensorDetectedBall == null) actualRightSensorDetectedBall = BallColorSet_Decode.NoBall;

        if(calculatedLeftSensorDetectedBall == null) calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall;
        if(calculatedRightSensorDetectedBall == null) calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall;

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
    public Pose passPose() {
        globalRobotPose = robot.getFollowerInstance().instance().getPose();
        return globalRobotPose;
    }
}