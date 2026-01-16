package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.farZoneCameraAdder;
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
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

import java.io.IOException;
import java.util.List;

@Config
@Autonomous(name = "NEW small triangle BLUE", group = "AAA")
public class SmallTriangleNew extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private Limelight3A limelight3A;
    public static MainConfig cfg;
    private boolean timeToFire, canSequence3, canSequence4,ballIsStuck;
    private boolean shouldFire;
    public static boolean isMoving;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime isFiringTimer = new ElapsedTime();
    ElapsedTime shouldJiggle = new ElapsedTime();
    ElapsedTime shouldFinish = new ElapsedTime();
    private boolean had_balls = false;

    public static boolean shouldMoveIntakeServo = false;

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
    public static double velocity = 1480;
    public static double angle = 266;
    public static double rotation = -20;
    public static int camId =23;
    /// --------------------------------------------------------
    private Pose starter = pose( 0.0, 0.0, 0.0); // Default Start Position (p0)
    private Pose small_triangle_shoot = pose(1, 0, 90); // Pose1: shooting position small triangle
    private Pose parkPose = pose(10, 15, 90);
    private Pose prepHPCollectPose = pose(15,39,145);
    private Pose fininshHPCollectPose = pose(2,46,90);
    private Pose halfTheWayHPCollectPose = pose(2,23,90);
    private Pose first_row_ready = pose(15, 52, 0); // Pose4: collect first row right
    private Pose first_row_done = pose(30, 52, 0); // Pose5: collect first row left
    private Pose lever = pose(38.31, -62.65, 0); // Pose6: lever pose
    private Pose second_row_ready = pose(15, 77, 0); // Pose7: collect second row right
    private Pose second_row_done = pose(30, 77, 0); // Pose8: colect second row left
    private Pose big_triangle_shoot = pose(1, -90, 0); // Pose9: shooting big triangle pose
    private Pose big_triangle_offset = pose(1, -70, 0); // Pose9: shooting big triangle pose
    private Pose third_row_ready = pose(15, 100, 0); // Pose10: collect third row right
    private Pose third_row_done = pose(30, 100, 0); // Pose11: collect third row left
    private Pose classifier_starter = pose(28.355210672213335, 119.64113250492127, 0); // Pose12: start position from sorter

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
                isMoving = ComplexFollower.instance().isBusy();
                HandleColors();
                if (ComplexFollower.followingForMS() > 2000 && ComplexFollower.getTarget().equals(fininshHPCollectPose) && !ComplexFollower.done()) ComplexFollower.interrupt();
                firingTurret(shouldFire);
                intakeChecks(shouldMoveIntakeServo);
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
        shouldFire = false;
        //teamSensitiveStuff();
    }

    @Override
    public void init_loop() {
        robot.init_loop();
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
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                    }
                }),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(2000),
                new GeneralAction(fireSortedBall),
                new DelayAction(700),
                new GeneralAction(fireSortedBall),
                new DelayAction(700),
                new GeneralAction(fireSortedBall),
                new DelayAction(700),
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),

                //second row of stuff
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(parkPose),
                new MoveAction(prepHPCollectPose),
                new DelayAction(150),
                new MoveAction(fininshHPCollectPose),
                new MoveAction(small_triangle_shoot),
                new GeneralAction(prepQueueToFireSortedBall),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(1500),
                new GeneralAction(fireSortedBall),
                new DelayAction(1200),
                new GeneralAction(fireSortedBall),
                new DelayAction(1200),
                new GeneralAction(fireSortedBall),
                new DelayAction(1200),
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),

                // mystery pick up
                new GeneralAction(turnOnIntakeServo),
                new StateAction("IntakeMotor","FULL"),
                new MoveAction(halfTheWayHPCollectPose),
                new DelayAction(500),
                new MoveAction(fininshHPCollectPose),
                new MoveAction(small_triangle_shoot),
                new GeneralAction(prepQueueToFireSortedBall),
                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(1500),
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
    public void firingTurret(boolean shouldFire) {
        double distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetX, cfg.targetY);
        double rotationToWallOdometry = - calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.targetXLeftPanel, cfg.targetYLeftPanel);
        rotationToWallOdometry += cfg.farZoneCameraAdder;
        if(rotationToWallOdometry < -30) rotationToWallOdometry += 360;

        if(shouldFire){
            // ----------------------- Power Stuff -----------------------

            //double targetVelocity = FAR_TARGET_VELOCITY;
            double targetVelocity = distanceToVelocityFunction(distanceToWallOdometry);
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Velocity)
                    .setTarget(targetVelocity);


            // ----------------------- Angle Stuff -----------------------

            double turretAngleVal = angle;
            turretAngleVal = clamp(turretAngleVal,262,324);
            robot.getServoComponent("TurretAngle")
                    .setTarget(turretAngleVal);

            // ----------------------- Rotation Stuff -----------------------

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

        actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors);
        actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if(!shouldRemoveBalls  && false) { // when not moving balls out of chambers they dont have permission to change to no ball
            if(actualLeftSensorDetectedBall != BallColorSet_Decode.NoBall)
                calculatedLeftSensorDetectedBall = actualLeftSensorDetectedBall;

            if(actualRightSensorDetectedBall != BallColorSet_Decode.NoBall)
                calculatedRightSensorDetectedBall = actualRightSensorDetectedBall;
        }
        else {
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
    protected void intakeChecks(boolean shouldCheck){
        int gateState = 0;
        if(shouldCheck) {
            if (!hasBallInRightChamber) gateState = 1; // first fill up left
            else if (!hasBallInLeftChamber) gateState = -1; // then right
            else gateState = -1; // then continue pointing to right for when you fire
        }
        else gateState = 1;
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
//    public void teamSensitiveStuff() {
//        if(targetY < 0) {
//            targetY = -targetY;
//        }
//        teamPipeline = 0;
//        currentTeamColor = TeamColor.Blue;
//        rotation = -21.5;
//    }
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Blue);
    }
    public void useCamera() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100); // poll 100 times per second
        limelight3A.start();

        LLResult llResult = limelight3A.getLatestResult();
        //llResult.getFiducialResults();
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            camId = fr.getFiducialId();
            //telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",, fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
        }
        if(camId < 21 || camId > 23) camId = 23;
    }
    public void convertPoses() {
        starter = convertPose(starter);
        parkPose = convertPose(parkPose);
    }
    public Pose convertPose(Pose pose) {
        return pose;
    }
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose(); //Math.toRadians
        return globalRobotPose;
    }
}