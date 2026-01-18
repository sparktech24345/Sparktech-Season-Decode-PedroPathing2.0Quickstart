package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.calculateDistance;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftSensorColorMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.pose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.ActionSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Config
@TeleOp(name="Main TeleOP Blue", group="AAA")
public class MainTeleOPBlue extends LinearOpMode {
    protected RobotController robot;
    protected VoltageSensor controlHubVoltageSensor;
    public static Pose farStart = pose(120, 24, 90); // no more reversing X
    public static double vp = 180;
    public static double vd = 18;
    public static double vf = 15;

    public static MainConfig cfg;

    /// ----------------- Target Pose Stuff ------------------

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

    /// ----------------- Limelight Stuff -----------------
    protected Limelight3A limelight3A = null;
    public static double distanceMeasuredCamera = 0;
    public static double rotationDegreesMeasuredCamera = 0;
    public static double lastRotationDegreesMeasuredCamera = 0;
    public static boolean shouldShootOnCamera = false;

    /// ----------------- Limelight Stuff -----------------
    private AnalogInput laserAnalog; // if < 110 then has ball
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    /// ----------------- Imu Stuff -----------------
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    /// ----------------- Odometry Stuff -----------------
    public static double distanceToWallOdometry = 0;
    public static double rotationToWallOdometry = 0;

    /// ----------------- Intake Priorities -----------------
    protected ElapsedTime sortingShootTimer = new ElapsedTime();
    protected ElapsedTime hasBeganShootingTimer = new ElapsedTime();
    protected ElapsedTime loweredGatesTimer = new ElapsedTime();
    public static boolean wantsToIntakeDriver = false;
    public static boolean shouldPassOneBallFor3InARow = false;
    public static boolean hasBallInOuttake = false;
    public static boolean wantsToFireWithIntake = false;
    public static boolean wantsToFireSortingWithIntake = false;
    public static boolean wantsToOutput = false;
    public static int gateState = 0;
    public static int lastGateState = 0;

    /// ----------------- Outtake Priorities -----------------
    public static double turretAngleOverride = 0;
    public static double turretVelocityOverride = 0;
    public static double timer1 = 500;
    public static double timer2 = 500;
    public static double timer3 = 800;
    public static double revUpTime = 1400;
    public static double timerToCloseGate = 300;
    public static double shootSortedTime = 1000;

    public static double TurretP = 0.025;
    public static double TurretD = 0.0015;
    public static double kVTurret = 0;
    public static double kATurret = 0;
    public static double kSTurret = 0;
    public static double TurretZero = 3;
    public static double rotationAdder = 0;

    public static boolean isTryingToFire = false;
    public static boolean needsToLowerGates = true;
    public static double fakeRotation = 0;
    public static Pose farPark = pose(117, 12, 90);

    public static double D2_velocityAdder = 0;
    public static double D2_rotationAdder = 0;

    public static double D2_velocityAdderMulti = 1;
    public static double D2_rotationAdderMulti = 1;

    protected void robotMainLoop() {
        // all of the code

        // processing
        processCameraStuff();
        processTargetStuff(robot.getCurrentPose(), cfg.targetX, cfg.targetY);
        distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetX, cfg.targetY);


        // this uses the processed target values
        rotationToWallOdometry = calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.usedTargetX, cfg.usedTargetY);
        RobotController.telemetry.addData("distance to wall", distanceToWallOdometry);
        RobotController.telemetry.addData("fakeRotation", fakeRotation);

        //if (rotationDegreesMeasuredCamera < 5 && rotationDegreesMeasuredCamera != 0) gamepad1.setLedColor(0, 254, 0, 250);
        //else gamepad1.setLedColor(254, 0, 0, 250);


        //colors
        HandleColors();

        // Choosing which values to use
        double usedDistance = 0;
        double neededAngleForTurretRotation = 0;
        if (shouldShootOnCamera && false) {
            //usedDistance = distanceMeasuredCamera;
            //neededAngleForTurretRotation += cameraAdder - rotationDegreesMeasuredCamera * cameraErrorMultiplier - robot.getMotorComponent("TurretRotateMotor").getPosition() * encoderMultiplier;
        }
        else {
            usedDistance = distanceToWallOdometry;
            neededAngleForTurretRotation -= rotationToWallOdometry; /// TODO this might be to be reversed in some ways
        }


        if (usedDistance > 2.9) neededAngleForTurretRotation += cfg.farZoneCameraAdder;
        if (shouldShootOnCamera) {
            if (Math.abs(robot.getMotorComponent("TurretRotateMotor").getVelocity()) < 0.2)
                rotationAdder += rotationDegreesMeasuredCamera;

            neededAngleForTurretRotation += rotationAdder;
        }

        neededAngleForTurretRotation += D2_rotationAdder * D2_rotationAdderMulti;
        if (neededAngleForTurretRotation < -30) neededAngleForTurretRotation += 360;

        // pose resets
        if (gamepad1.dpadLeftWasPressed()) {
            ComplexFollower.instance().setPose(pose(0, 0, 0));
        }
        if (gamepad1.dpadRightWasPressed()) {
            ComplexFollower.instance().setPose(farPark);
        }

        // Driver Intake
        if (robot.getKey("A1").ExecuteOnPress){
            wantsToIntakeDriver = !wantsToIntakeDriver;
            shouldPassOneBallFor3InARow = wantsToIntakeDriver; // true if true and will be then turned false, false if false

            wantsToFireWithIntake = false;
            wantsToFireSortingWithIntake = false;
        }

        // Driver preparing for shooting
        if (robot.getKey("Y1").ExecuteOnPress){
            isTryingToFire = !isTryingToFire;

            shouldPullFromQueue = false;
            sortingShootTimer.reset();
            hasBeganShootingTimer.reset();
        }
        // Driver actual firing with sorting
        if (robot.getKey("B1").ExecuteOnPress){
            wantsToFireSortingWithIntake = !wantsToFireSortingWithIntake;

            wantsToFireWithIntake = false;
            wantsToIntakeDriver = false;
        }

        // Driver Actual Shooting
        if (robot.getKey("X1").ExecuteOnPress){
            wantsToFireWithIntake = !wantsToFireWithIntake;
            hasBallInOuttake = false;

            wantsToFireSortingWithIntake = false;
            wantsToIntakeDriver = false;

        }
        needsToLowerGates = robot.getKey("X1").ExecuteOnPress;

        // Driver Outputting
        if (robot.getKey("RIGHT_BUMPER1").ExecuteOnPress) wantsToOutput = !wantsToOutput;

        if (robot.getKey("DPAD_UP1").IsToggledOnPress)
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Power)
                    .setTarget(1);

        // ====================== Sorting Stuff ======================

        if (robot.getKey("RIGHT_BUMPER2").ExecuteOnPress) {
            ballColorQueue.add(BallColorSet_Decode.Purple);
            gamepad2.setLedColor(254, 0, 254, 1000000);
        }
        if (robot.getKey("LEFT_BUMPER2").ExecuteOnPress) {
            ballColorQueue.add(BallColorSet_Decode.Green);
            gamepad2.setLedColor(0, 254, 0, 1000000);
        }
        if (robot.getKey("B2").ExecuteOnPress) {
            ballColorQueue.clearQueue();
            gamepad2.setLedColor(254, 254, 254, 1000000);
        }

        // Enable / Disable camera

        flashSensors(gamepad1.right_trigger > 0.4);

        // Slowdown
        if (robot.getKey("LEFT_BUMPER1").ExecuteOnPress) {
            if (robot.getKey("LEFT_BUMPER1").IsToggledOnPress) {
                DriveTrain.setSlowdown(0.3);
            } else DriveTrain.setSlowdown(1);
        }

        // Adders

        if (robot.getKey("DPAD_UP2").ExecuteOnPress) {
            D2_velocityAdder += 10;
        }

        if (robot.getKey("DPAD_DOWN2").ExecuteOnPress) {
            D2_velocityAdder -= 10;
        }

        if (robot.getKey("DPAD_DOWN2").IsHeld && robot.getKey("DPAD_UP2").IsHeld) {
            D2_velocityAdder = 0;
        }

        if (robot.getKey("DPAD_RIGHT2").ExecuteOnPress) {
            D2_rotationAdder += 1;
        }

        if (robot.getKey("DPAD_LEFT2").ExecuteOnPress) {
            D2_rotationAdder -= 1;
        }

        if (robot.getKey("DPAD_RIGHT2").IsHeld && robot.getKey("DPAD_LEFT2").IsHeld) {
            D2_rotationAdder = 0;
        }

        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Decision Making Code ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        if (wantsToOutput || wantsToFireWithIntake || (wantsToFireSortingWithIntake && isTryingToFire)) // only actually affect balls when you can output balls so you can count them even if they are stying on the hole and are undetected for a time
            shouldRemoveBalls = true;
        else shouldRemoveBalls = false;


        // Intake stuff
        int intakeState = 0; // if -1 then output if 0 then do nothing if 1 then intake
        ; // if -1 then gate To The Left if 0 then do block if 1 then to the right
        if (!wantsToIntakeDriver  && !wantsToFireWithIntake && !wantsToOutput && !wantsToFireSortingWithIntake) {
            intakeState = 0;
            //gateState = 0; // if doing nothing
        }
        if (wantsToOutput) {
            wantsToIntakeDriver = wantsToFireSortingWithIntake = wantsToFireWithIntake = false; // make all of them false for override
            intakeState = -1;
            if (hasBallInRightChamber) gateState = 1; // open to the right so you can outtake
            else if (hasBallInLeftChamber) gateState = -1; // left is lower priority but checked
            //else gateState = 0; // output last one fast
        }
        if (wantsToIntakeDriver  || wantsToFireWithIntake || wantsToFireSortingWithIntake) { // for now keep them in the same bucket
            intakeState = 1;
            if (wantsToFireSortingWithIntake) {
                if (!hasBallInRightChamber) gateState = 1; // first fill up right
                else if (!hasBallInLeftChamber) gateState = -1; // then left
                else gateState = 0; // only for sorted
                //else gateState = lastGateState;
            }
            if(wantsToIntakeDriver){
                gateState = 1 ;// redirect to right;
                if(shouldPassOneBallFor3InARow && hasBallInRightChamber && !hasBallInOuttake){
                    shouldPassOneBallFor3InARow = false;
                    hasBallInOuttake = true;
                    loweredGatesTimer.reset();
                    robot.executeNow(new ActionSequence(
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate),
                            new StateAction("RightGateServo", "CLOSED")
                    ));
                }
            }
        }
        lastGateState = gateState;

        if ((robot.getKey("X2").IsHeld)) gateState = -1; // left
        if ((robot.getKey("Y2").IsHeld)) gateState = 1; // right
        if ((robot.getKey("A2").IsHeld)) gateState = 0; // right

        switch (intakeState) {
            case -1:
                robot.executeNow(new StateAction("IntakeMotor", "FULL_REVERSE"));
                break;

            case 0:
                robot.executeNow(new StateAction("IntakeMotor", "OFF"));
                break;

            case 1:
                robot.executeNow(new StateAction("IntakeMotor", "FULL"));
                break;
        }
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

        //firing logic
        if (!wantsToFireSortingWithIntake) {
            if (wantsToFireWithIntake) { // if normal is active
                if (needsToLowerGates) {
                    if (usedDistance > 2.9) {
                        needsToLowerGates = false; // to not infi repeat
                        robot.executeNow(new ActionSequence(
                                new StateAction("LeftGateServo", "OPEN"),
                                new DelayAction(timerToCloseGate),
                                new StateAction("LeftGateServo", "CLOSED"),

                                new DelayAction(timer1),

                                new StateAction("LeftGateServo", "OPEN"),

                                new DelayAction(timer2),

                                new StateAction("RightGateServo", "OPEN")
                        ));
                    }
                    else {
                        needsToLowerGates = false; // to not infi repeat
                        robot.executeNow(new ActionSequence(
                        new StateAction("RightGateServo", "OPEN"),
                        new DelayAction(timer3),
                        new StateAction("LeftGateServo", "OPEN")
                        ));
                    }

                }
            }
            else if (!needsToLowerGates) { // if neither is active
                needsToLowerGates = true; // to not infi repeat, will not trigger the one above due to wantToFireWithIntake
                if (wantsToOutput) {
                    robot.executeNow(new StateAction("RightGateServo", "OPEN"));
                    robot.executeNow(new StateAction("LeftGateServo", "OPEN"));
                }
                else if(loweredGatesTimer.milliseconds() > 500){
                    robot.executeNow(new StateAction("RightGateServo", "CLOSED"));
                    robot.executeNow(new StateAction("LeftGateServo", "CLOSED"));
                }
            }
        }
        else {
            if (wantsToFireWithIntake) { // wtf why are they both active
                gamepad1.rumble(1, 1, 200);
            }
            else { // if sorting firing is active

                if (sortingShootTimer.milliseconds() > shootSortedTime && isTryingToFire && hasBeganShootingTimer.milliseconds() > revUpTime) { // D2 failsafe
                    shouldPullFromQueue = true;
                    sortingShootTimer.reset();
                }

                if (shouldPullFromQueue && isTryingToFire) {
                    ballToFire = ballColorQueue.pull();
                    shouldPullFromQueue = false;

                    if (ballToFire == calculatedRightSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
                        robot.executeNow(new ActionSequence(
                                new StateAction("RightGateServo", "OPEN"),
                                new DelayAction(timerToCloseGate),
                                new StateAction("RightGateServo", "CLOSED")
                        ));
                    }
                    else if (ballToFire == calculatedLeftSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
                        robot.executeNow(new ActionSequence(
                                new StateAction("LeftGateServo", "OPEN"),
                                new DelayAction(timerToCloseGate),
                                new StateAction("LeftGateServo", "CLOSED")
                        ));
                    }
                    else
                        if((calculatedRightSensorDetectedBall  != ballToFire && calculatedLeftSensorDetectedBall != ballToFire))
                            ballToFire = BallColorSet_Decode.NoBall; // that means that both thins have the same ball or no ball case in wich no problem
                    else ballColorQueue.push(ballToFire); // add it back if it was a fake call
                }


            }
        }

        // Outtake Stuff
        double targetVelocity = 0;
        if (isTryingToFire) {
            // ----------------------- Power Stuff -----------------------

            targetVelocity = distanceToVelocityFunction(usedDistance);
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Velocity)
                    .setTarget((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                    .setVelocityCoefficients(vp,0,vd,vf);


            // ----------------------- Angle Stuff -----------------------

            double turretAngleVal = distanceToAngleFunction(usedDistance);
            turretAngleVal = clamp(turretAngleVal, 262, 324);
            robot.getServoComponent("TurretAngle")
                    .setTarget((eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal));


            // ----------------------- Rotation Stuff -----------------------

            if (robot.getKey("DPAD_DOWN1").IsToggledOnPress) neededAngleForTurretRotation = 0; // stop rotation if ododmetry dies bad
            robot.getTurretComponent("TurretRotateMotor")
                    .setFeedforwardCoefficients(kVTurret,kATurret,kSTurret)
                    .setTarget(neededAngleForTurretRotation)
                    .setPositionCoefficients(TurretP, 0, TurretD, TurretZero) // only enable for tunning porposes
            ;

        }
        else {
            rotationAdder = 0;

            if (!(robot.getKey("DPAD_UP1").IsToggledOnPress)) {
                robot.getMotorComponent("TurretSpinMotor")
                        .setOperationMode(MotorComponent.MotorModes.Power)
                        .setTarget(0);
            }

            double turretAngleVal = distanceToAngleFunction(usedDistance);
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT")); // go to default position

            robot.getMotorComponent("TurretRotateMotor")
                    .setTarget(0);


        }

        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Telemetry and Overrides ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        RobotController.telemetry.addData("robot rotation in degrees", Math.toDegrees(robot.getCurrentPose().getHeading()));
        RobotController.telemetry.addData("robot Y", robot.getCurrentPose().getY());
        RobotController.telemetry.addData("robot X", robot.getCurrentPose().getX());
        RobotController.telemetry.addData("is outtake Overriden", robot.getMotorComponent("TurretSpinMotor").isOverriden());
        RobotController.telemetry.addData("target velocity", targetVelocity);
        RobotController.telemetry.addData("actual Velocity", robot.getMotorComponent("TurretSpinMotor").getVelocity());
        RobotController.telemetry.addData("SPEED", robot.getMotorComponent("TurretSpinMotor").getPower());
        RobotController.telemetry.addData("CURRENT", robot.getMotorComponent("TurretSpinMotor").getCurrent());
        RobotController.telemetry.addData("Intake Current", robot.getMotorComponent("IntakeMotor").getCurrent());
        RobotController.telemetry.addData("Current Rotation", robot.getMotorComponent("TurretRotateMotor").getPosition());
        RobotController.telemetry.addData("Target Rotation", neededAngleForTurretRotation);
        RobotController.telemetry.addData("D2 velocity adder", D2_velocityAdder * D2_velocityAdderMulti);
        RobotController.telemetry.addData("D2 rotation adder", D2_rotationAdder * D2_rotationAdderMulti);
        RobotController.telemetry.addData("shouldPassOneBallFor3InARow",shouldPassOneBallFor3InARow);
        RobotController.telemetry.addData("hasBallInRightChamber",hasBallInRightChamber);
        RobotController.telemetry.addData("hasBallInOuttake",hasBallInOuttake);
        ballColorQueue.spitOutQueueInTelemetry();
    }


    // ============================ Init Stuff ============================


    @Override
    public void runOpMode() {
        // init
        setStuffToDefault();

        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {

                robotMainLoop();
            }
        };

        robot.init(OpModes.TeleOP);
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        InitOtherStuff(teamPipeline);
        robot.UseDefaultMovement();
        makeConfig();

        while (opModeInInit()) {
            robot.init_loop();
        }
        ComplexFollower.instance().setPose(globalRobotPose);
        //ComplexFollower.setStartingPose(globalRobotPose);

        while (opModeIsActive()) {
            // loop
            robot.loop();
        }
        passPose();
    }

    public void setStuffToDefault() { // occasionally copy paste declarations here so we don't have surprises
        ballCounter = 0;
        shouldShootOnCamera = false;
        shouldRemoveBalls = false;
        shouldPullFromQueue = false;
        shouldPassOneBallFor3InARow = false;
        hasBallInOuttake = false;
        if (ballColorQueue == null) ballColorQueue = new BallColorQueue();
        ballColorQueue.clearQueue();
        ballToFire = BallColorSet_Decode.NoBall;
        sortingShootTimer.reset();
        loweredGatesTimer.reset();
        hasBeganShootingTimer.reset();

        /// ----------------- Odometry Stuff -----------------
        distanceToWallOdometry = 0;
        rotationToWallOdometry = 0;
        rotationAdder = 0;
        lastRotationDegreesMeasuredCamera = 0;

        /// ----------------- Intake Priorities -----------------

        wantsToIntakeDriver = false;
        wantsToFireSortingWithIntake = false;
        wantsToFireWithIntake = false;
        wantsToOutput = false;

        /// ----------------- Outtake Priorities -----------------
        turretAngleOverride = 0;
        turretVelocityOverride = 0;

        isTryingToFire = false;
        needsToLowerGates = true;
        // ------------------ Driver 2 adders --------------------
        D2_velocityAdder = 0;
        D2_rotationAdder = 0;
    }

    public void InitOtherStuff(int limelightPipeline) {
        //limelight stuff
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(limelightPipeline);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100); // poll 100 times per second
        limelight3A.start();

        //other stuff like color sensor
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        gamepad1.setLedColor(254, 254, 254, 1000000);
        gamepad2.setLedColor(254, 0, 0, 1000000);
    }


    // ============================ Color Stuff ============================

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


    // ============================ Camera Stuff ============================

    public void processCameraStuff() {
        LLResult llResult = limelight3A.getLatestResult();

        double tempRotation = llResult.getTx();
        // rotation
        if (tempRotation == 0 || Math.abs(tempRotation) > 30)
            rotationDegreesMeasuredCamera = lastRotationDegreesMeasuredCamera;
        else rotationDegreesMeasuredCamera = tempRotation;
        lastRotationDegreesMeasuredCamera = rotationDegreesMeasuredCamera;

        // distance

        double targetArea = llResult.getTa();

        RobotController.telemetry.addData("areaPercentage", targetArea);
        double a = 8.60403612;
        double b = -0.0119936722;

        distanceMeasuredCamera = Math.log(targetArea / a) / b;
    }

    // ============================ Odometry Stuff ============================

    public static double calculateHeadingAdjustment(Pose robotPose, double robotHeadingDeg, double targetX, double targetY) {
        // Current robot position
        double x = robotPose.getX();
        double y = robotPose.getY(); // don'// invert Y unless your coordinate system specifically requires it
        // Vector from robot to target
        double dx = targetX - x;
        double dy = targetY - y;

        // Angle from robot to target (in radians → degrees)
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Normalize to [0, 360)
        if (targetAngleDeg<0) targetAngleDeg += 360;

        // Robot heading in degrees
        // Assuming robotPose.getHeading() is in radians, 0° = facing +X, increases counterclockwise
        //double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Normalize to [0, 360)
        if (robotHeadingDeg<0) robotHeadingDeg += 360;

        // Calculate smallest signed angle difference: [-180, 180]
        double angleDiff = targetAngleDeg - robotHeadingDeg;
        angleDiff = ((angleDiff + 540) % 360) - 180;  // neat trick for wrapping to [-180, 180]

        // Positive = target to the robot's right (clockwise turn), negative = to the left

        return angleDiff;
    }
    // pure angle not accounting for robot direction, DO NOT use for turret, use for target selection degrees
    public static double angleFromTargetToRobot(Pose pose, double targetX, double targetY) {
        // from pose to point
        double robotX = pose.getX();
        double robotY = pose.getY();

        double dx = robotX - targetX;
        double dy = robotY - targetY;

        // atan2(dx, dy) gives an angle in radians in the range [-PI, PI]
        // matching:
        // same X -> 0 or 180
        // same Y -> +90 or -90
        // left negative, right positive
        double angleRadians = Math.atan2(dx, dy);

        return Math.toDegrees(angleRadians) + 90;
    }

    public static void processTargetStuff(Pose pose, double targetX, double targetY) {
        double degrees = Math.abs(angleFromTargetToRobot(pose, targetX, targetY)); // Abs value so that it works for red and blue

        if (degrees < 25) { // is lower part of scorer
            cfg.usedTargetX = cfg.targetXRightPanel;
            cfg.usedTargetY = cfg.targetYRightPanel;
        }
        else if (degrees > 70) {  // is right upper part of scorer
            cfg.usedTargetX = cfg.targetXLeftPanel;
            cfg.usedTargetY = cfg.targetYLeftPanel;
        }
        else { // is in the middle or somewhere weird
            cfg.usedTargetX = cfg.targetXCenter;
            cfg.usedTargetY = cfg.targetYCenter;
        }


        //usedTargetX = targetXCenter;
        //usedTargetY = targetYCenter;

        RobotController.telemetry.addData("Calculated Rotation From Robot", degrees);
    }

    public static double calculateDistanceToWallInMeters(Pose robotPose, double targetX, double targetY) {
        return calculateDistance(robotPose, new Pose(targetX, targetY, 0), true);
    }


    // ============================ Weird Stuff ============================
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose(); //Math.toRadians
        return globalRobotPose;
    }
    
    protected ElapsedTime flashingTimer = new ElapsedTime();
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Blue);
    }
    public void flashSensors(boolean isFlashing) {
        if (flashingTimer.milliseconds() % 200 > 100) {
            robot.getColorSensorComponent("colorSensorRight").useSensorLight(false);
            robot.getColorSensorComponent("colorSensorLeft").useSensorLight(false);
        }
        else {
            robot.getColorSensorComponent("colorSensorRight").useSensorLight(true);
            robot.getColorSensorComponent("colorSensorLeft").useSensorLight(true);
        }
        if (!isFlashing) {
            robot.getColorSensorComponent("colorSensorRight").useSensorLight(true);
            robot.getColorSensorComponent("colorSensorLeft").useSensorLight(true);
        }
    }

}
