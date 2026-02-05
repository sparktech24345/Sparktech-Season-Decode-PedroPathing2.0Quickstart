package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.calculateDistance;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
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

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.ActionSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.Drivers;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Config
@TeleOp(name="Main TeleOp Blue", group="AAA")
public class MainTeleOpBlue extends LinearOpMode {
    protected RobotController robot;
    protected VoltageSensor controlHubVoltageSensor;
    public static Pose farStart = pose(120, 24, 90);
    public static double vp = 0.0055;
    public static double velp = 195;
    public static double vd =0;
    public static double veld =25;
    public static double vf = 0.00045;
    public static double velf = 15;
    public static double vMultiplier = 1;

    public static MainConfig cfg;
    public static Drivers driver = Drivers.Teo;

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
    public static boolean wantsToTempOutputIntake = false;
    public static boolean shouldResetRightSensorBall = false;
    BallColorQueue ballColorQueue = new BallColorQueue();


    protected Limelight3A limelight3A = null;
    public static double distanceMeasuredCamera = 0;
    public static double rotationDegreesMeasuredCamera = 0;
    public static double lastRotationDegreesMeasuredCamera = 0;
    public static boolean shouldShootOnCamera = false;
    public static double angleDecreaseValue = 10;
    public static double rightSideAngleBias = -2;
    public static double velocityDeltaCompensation = 80;

    private AnalogInput laserAnalog;
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    public static double distanceToWallOdometry = 0;
    public static double rotationToWallOdometry = 0;


    protected ElapsedTime sortingShootTimer = new ElapsedTime();
    protected ElapsedTime resetLeftBallColorTimer = new ElapsedTime();
    public static boolean wantsToIntakeDriver = false;
    public static boolean hasBallInOuttake = false;
    public static boolean wantsToFireWithIntake = false;
    public static boolean isInSortedMode = false;
    public static boolean wantsToOutput = false;
    public static boolean hasJustBeganFiring = false;
    public static int intakeGateState = 0;
    public static int outtakeGatesState = -1;
    public static int intakeState = 0;
    public static int lastGateState = 0;

    public static double turretAngleOverride = 0;
    public static double turretVelocityOverride = 0;
    public static double timer1 = 550;
    public static double timer5 = 600;
    public static double timer6 = 400;
    public static double outtakeReversingTime = 180;
    public static double timer2 = 700;
    public static double timer3 = 100;
    public static double timer4 = 300;
    public static double revUpTime = 1400;
    public static double timerToCloseGate = 300;
    public static double shootSortedTime = 800;

    public static double TurretP = 0.025;
    public static double TurretD = 0.0015;
    public static double kVTurret = 0;
    public static double kATurret = 0;
    public static double kSTurret = 0;
    public static double TurretZero = 2;
    public static double rotationAdder = 0;
    public static boolean isTryingToFire = false;
    public static boolean isMovingOuttakeGates = false;
    public static boolean hasSwitchedIntakeState = false;
    public static double fakeRotation = 0;
    public static Pose farPark = pose(117, 12, 90);

    public static double D2_velocityAdder = 0;
    public static double D2_rotationAdder = 0;

    public static double D2_velocityAdderMulti = 1;
    public static double D2_rotationAdderMulti = 1;


    public static boolean shouldShootWithoutTurret = false;
    public static boolean shouldForceOuttake = false;
    public static boolean wantsToFireWithIntakeUnsortedInSortingMode = false;
    public static double forcedOuttakeSpeed = 0.7;
    public static double oneTunnelDistance = 1.65;

    protected void robotMainLoop() {

        processCameraStuff();
        processTargetStuff(robot.getCurrentPose(), cfg.targetX, cfg.targetY);
        distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.targetX, cfg.targetY);


        rotationToWallOdometry = calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), cfg.usedTargetX, cfg.usedTargetY);
        RobotController.telemetry.addData("distance to wall", distanceToWallOdometry);
        RobotController.telemetry.addData("fakeRotation", fakeRotation);

        HandleColors();


        double usedDistance = 0;
        double neededAngleForTurretRotation = 0;
        usedDistance = distanceToWallOdometry;
        neededAngleForTurretRotation -= rotationToWallOdometry;


        if (usedDistance > 2.9) neededAngleForTurretRotation += cfg.farZoneCameraAdder;
        if (shouldShootOnCamera) {
            if (Math.abs(robot.getMotorComponent("TurretRotateMotor").getVelocity()) < 0.2)
                rotationAdder += rotationDegreesMeasuredCamera;

            neededAngleForTurretRotation += rotationAdder;
        }

        neededAngleForTurretRotation += D2_rotationAdder * D2_rotationAdderMulti;
        if (neededAngleForTurretRotation > -30) neededAngleForTurretRotation += rightSideAngleBias;
        if (neededAngleForTurretRotation < -30) neededAngleForTurretRotation += 360;


        if (robot.getKey("DPAD_LEFT1").ExecuteOnPress) {
            ComplexFollower.instance().setPose(pose(0, 0, 0));
            D2_velocityAdder = 0;
            D2_rotationAdder = 0;
        }
        shouldShootWithoutTurret = robot.getKey("DPAD_DOWN1").IsToggledOnPress;
        shouldForceOuttake = robot.getKey("DPAD_UP1").IsToggledOnPress;

        if (robot.getKey("DPAD_RIGHT1").ExecuteOnPress) {
            robot.executeNow(new ActionSequence(
                    new GeneralAction(() -> wantsToTempOutputIntake = true),
                    new DelayAction(60),
                    new GeneralAction(() -> wantsToTempOutputIntake = false)
            ));
        }


        if (robot.getKey("LEFT_BUMPER1").ExecuteOnPress) {
            wantsToOutput = !wantsToOutput;

            wantsToFireWithIntake = false;
            wantsToIntakeDriver = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        if (robot.getKey("RIGHT_BUMPER1").ExecuteOnPress) {
            wantsToIntakeDriver = !wantsToIntakeDriver;

            wantsToFireWithIntake = false;
            wantsToOutput = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        if (robot.getKey("RIGHT_TRIGGER1").ExecuteOnPress) {
            wantsToFireWithIntake = !wantsToFireWithIntake;
            hasBallInOuttake = false;
            hasJustBeganFiring = true;

            wantsToIntakeDriver = false;
            wantsToOutput = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        if (robot.getKey("LEFT_TRIGGER1").ExecuteOnPress) isTryingToFire = !isTryingToFire;


        if (robot.getKey("Y1").ExecuteOnPress) {
            isInSortedMode = !isInSortedMode;
        }
        if (isInSortedMode) gamepad1.setLedColor(0, 0, 255, 30000);
        else gamepad1.setLedColor(255, 0, 0, 30000);
        if (robot.getKey("X1").ExecuteOnPress) {
            wantsToFireWithIntakeUnsortedInSortingMode = !wantsToFireWithIntakeUnsortedInSortingMode;
            hasJustBeganFiring = true;

            wantsToIntakeDriver = false;
            wantsToOutput = false;
            wantsToFireWithIntake = false;
            hasSwitchedIntakeState = true;
        }

        if (robot.getKey("B1").ExecuteOnPress) {
            if (robot.getKey("B1").IsToggledOnPress) {
                DriveTrain.setSlowdown(0.3);
            } else DriveTrain.setSlowdown(1);
        }

        if (robot.getKey("A1").ExecuteOnPress) {
            hasBallInOuttake = false;
        }


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

        if (robot.getKey("RIGHT_TRIGGER2").ExecuteOnPress) {
            if (calculatedRightSensorDetectedBall == BallColorSet_Decode.NoBall)
                calculatedRightSensorDetectedBall = BallColorSet_Decode.Purple;
            else calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall;
        }
        if (robot.getKey("LEFT_TRIGGER2").ExecuteOnPress) {
            if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.NoBall)
                calculatedLeftSensorDetectedBall = BallColorSet_Decode.Purple;
            else calculatedLeftSensorDetectedBall = BallColorSet_Decode.NoBall;
        }


        if (hasSwitchedIntakeState) {
            outtakeGatesState = -1;
            isMovingOuttakeGates = false;
            robot.executeNow(new StateAction("RightGateServo", "CLOSED"));
            robot.executeNow(new StateAction("LeftGateServo", "CLOSED"));
        }


        if (!isInSortedMode || wantsToFireWithIntakeUnsortedInSortingMode) {
            if (wantsToIntakeDriver) {
                intakeGateState = 1;
                if (hasBallInRightChamber) intakeGateState = -1;
            } else intakeGateState = lastGateState;
            lastGateState = intakeGateState;


            if (wantsToIntakeDriver && false) {
                if (hasBallInRightChamber && !hasBallInOuttake) {
                    hasBallInOuttake = true;
                    isMovingOuttakeGates = true;
                    robot.executeNow(new ActionSequence(
                            new GeneralAction(new Runnable() {
                                @Override
                                public void run() {
                                    resetLeftBallColorTimer.reset();
                                    shouldResetRightSensorBall = true;
                                }
                            }),
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate),
                            new StateAction("RightGateServo", "CLOSED")
                    ));
                }
            }

            if (((wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) && hasJustBeganFiring)) {
                isMovingOuttakeGates = true;
                if (usedDistance > 2.9) {
                    robot.executeNow(new ActionSequence(
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate),
                            new StateAction("RightGateServo", "CLOSED"),

                            new DelayAction(timer1),

                            new StateAction("RightGateServo", "OPEN"),

                            new DelayAction(timer2),

                            new StateAction("LeftGateServo", "OPEN")
                    ));
                    hasJustBeganFiring = false;
                } else {
                    robot.executeNow(new ActionSequence(

                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate),
                            new StateAction("RightGateServo", "CLOSED"),
                            new DelayAction(timer3),
                            new StateAction("LeftGateServo", "OPEN"),
                            new DelayAction(timer4),
                            new StateAction("RightGateServo", "OPEN")
                    ));
                    hasJustBeganFiring = false;
                }
            }


        } else {

            if (!hasBallInRightChamber) intakeGateState = 1;
            else if (!hasBallInLeftChamber) intakeGateState = -1;
            else intakeGateState = 0;

            if (sortingShootTimer.milliseconds() > shootSortedTime && wantsToFireWithIntake) {
                shouldPullFromQueue = true;
                sortingShootTimer.reset();
            }

            if (shouldPullFromQueue) {
                ballToFire = ballColorQueue.pull();

                if (ballToFire == calculatedRightSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
                    isMovingOuttakeGates = true;
                    robot.executeNow(new ActionSequence(
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate),
                            new StateAction("RightGateServo", "CLOSED")
                    ));
                } else if (ballToFire == calculatedLeftSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall) {
                    isMovingOuttakeGates = true;
                    robot.executeNow(new ActionSequence(
                            new StateAction("LeftGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate),
                            new StateAction("LeftGateServo", "CLOSED")
                    ));
                } else if ((calculatedRightSensorDetectedBall != ballToFire && calculatedLeftSensorDetectedBall != ballToFire))
                    ballToFire = BallColorSet_Decode.NoBall;
                ballToFire = BallColorSet_Decode.NoBall;
            }
            shouldPullFromQueue = false;
        }


        if (!hasBallInLeftChamber && !hasBallInRightChamber) outtakeGatesState = -1;

        if (wantsToOutput) {
            intakeState = -1;
            isMovingOuttakeGates = false;
            outtakeGatesState = 1;
            if (hasBallInRightChamber) intakeGateState = 1;
            else if (hasBallInLeftChamber) intakeGateState = -1;
        } else if (wantsToIntakeDriver) intakeState = 1;
        else if (wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode)
            intakeState = 2;
        else intakeState = 0;

        if (wantsToTempOutputIntake)
            intakeState = -1;

        hasSwitchedIntakeState = false;


        if ((robot.getKey("X2").IsHeld)) intakeGateState = -1;
        if ((robot.getKey("Y2").IsHeld)) intakeGateState = 1;
        if ((robot.getKey("A2").IsHeld)) intakeGateState = 0;
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
            case 2:
                robot.executeNow(new StateAction("IntakeMotor", "FIRING_POWER"));
                break;
        }
        switch (intakeGateState) {
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
        if (!isMovingOuttakeGates) {
            switch (outtakeGatesState) {
                case -1:
                    robot.executeNow(new StateAction("RightGateServo", "CLOSED"));
                    robot.executeNow(new StateAction("LeftGateServo", "CLOSED"));
                    break;

                case 0:
                    break;

                case 1:
                    robot.executeNow(new StateAction("RightGateServo", "OPEN"));
                    robot.executeNow(new StateAction("LeftGateServo", "OPEN"));
                    break;
            }
        }

        shouldRemoveBalls = wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode || wantsToOutput;

        double targetVelocity = 0;
        if (isTryingToFire) {

            targetVelocity = distanceToVelocityFunction(usedDistance) * vMultiplier;
            if (!shouldForceOuttake) {
                if (Math.abs(targetVelocity - robot.getMotorComponent("TurretSpinMotor").getVelocity()) <= 80) {
                    robot.getMotorComponent("TurretSpinMotor")
                            .setOperationMode(MotorComponent.MotorModes.Velocity)
                            .setTarget((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                            .setVelocityCoefficients(velp, 0, veld, velf);
                } else {
                    robot.getMotorComponent("TurretSpinMotor")
                            .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                            .setTarget((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                            .setAccelerationVelocityCoefficients(vp, 0, vd, vf);
                    ;
                }

            } else {
                robot.getMotorComponent("TurretSpinMotor")
                        .setOperationMode(MotorComponent.MotorModes.Power)
                        .setTarget(forcedOuttakeSpeed);
            }

            if (Math.abs(robot.getMotorComponent("TurretSpinMotor").getVelocity() - targetVelocity) <= 21)
                gamepad1.rumble(0.4, 0.4, 100);

            double turretAngleVal = distanceToAngleFunction(usedDistance);
            turretAngleVal = clamp(turretAngleVal, 262, 315);
            robot.getServoComponent("TurretAngle")
                    .setTarget((eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal));


            if (shouldShootWithoutTurret) neededAngleForTurretRotation = 0;
            robot.getMotorComponent("TurretRotateMotor")
                    .setTarget(neededAngleForTurretRotation)
                    .setPositionCoefficients(TurretP, 0, TurretD, TurretZero)
            ;

        } else {
            rotationAdder = 0;

            if (!shouldForceOuttake) {
                robot.getMotorComponent("TurretSpinMotor")
                        .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                        .setTarget(0);
            } else {
                robot.getMotorComponent("TurretSpinMotor")
                        .setOperationMode(MotorComponent.MotorModes.Power)
                        .setTarget(forcedOuttakeSpeed);
            }


            double turretAngleVal = distanceToAngleFunction(usedDistance);
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT"));

            robot.getMotorComponent("TurretRotateMotor")
                    .setTarget(0);


        }
    }

    @Override
    public void runOpMode() {
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

        while (opModeIsActive()) {
            robot.loop();
        }
        passPose();
    }

    public void setStuffToDefault() {
        ballCounter = 0;
        shouldShootOnCamera = false;
        shouldRemoveBalls = false;
        shouldPullFromQueue = false;
        hasBallInOuttake = false;
        wantsToTempOutputIntake = false;
        shouldResetRightSensorBall = false;
        if (ballColorQueue == null) ballColorQueue = new BallColorQueue();
        ballColorQueue.clearQueue();
        ballToFire = BallColorSet_Decode.NoBall;
        sortingShootTimer.reset();


        distanceToWallOdometry = 0;
        rotationToWallOdometry = 0;
        rotationAdder = 0;
        lastRotationDegreesMeasuredCamera = 0;


        wantsToIntakeDriver = false;
        isInSortedMode = false;
        hasSwitchedIntakeState = false;
        wantsToFireWithIntake = false;
        wantsToOutput = false;
        isMovingOuttakeGates = false;
        hasJustBeganFiring = false;

        turretAngleOverride = 0;
        turretVelocityOverride = 0;

        isTryingToFire = false;
        D2_velocityAdder = 0;
        D2_rotationAdder = 0;
    }

    public void InitOtherStuff(int limelightPipeline) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(limelightPipeline);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100);
        limelight3A.start();

        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        gamepad1.setLedColor(254, 254, 254, 1000000);
        gamepad2.setLedColor(254, 0, 0, 1000000);
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


    public void processCameraStuff() {
        LLResult llResult = limelight3A.getLatestResult();

        double tempRotation = llResult.getTx();
        if (tempRotation == 0 || Math.abs(tempRotation) > 30)
            rotationDegreesMeasuredCamera = lastRotationDegreesMeasuredCamera;
        else rotationDegreesMeasuredCamera = tempRotation;
        lastRotationDegreesMeasuredCamera = rotationDegreesMeasuredCamera;

        double targetArea = llResult.getTa();

        RobotController.telemetry.addData("areaPercentage", targetArea);
        double a = 8.60403612;
        double b = -0.0119936722;

        distanceMeasuredCamera = Math.log(targetArea / a) / b;
    }


    public static double calculateHeadingAdjustment(Pose robotPose, double robotHeadingDeg, double targetX, double targetY) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        double dx = targetX - x;
        double dy = targetY - y;

        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        if (targetAngleDeg<0) targetAngleDeg += 360;

        if (robotHeadingDeg<0) robotHeadingDeg += 360;

        double angleDiff = targetAngleDeg - robotHeadingDeg;
        angleDiff = ((angleDiff + 540) % 360) - 180;

        return angleDiff;
    }
    public static double angleFromTargetToRobot(Pose pose, double targetX, double targetY) {
        double robotX = pose.getX();
        double robotY = pose.getY();

        double dx = robotX - targetX;
        double dy = robotY - targetY;

        double angleRadians = Math.atan2(dx, dy);

        return Math.toDegrees(angleRadians) + 90;
    }

    public static void processTargetStuff(Pose pose, double targetX, double targetY) {
        double degrees = Math.abs(angleFromTargetToRobot(pose, targetX, targetY));

        if (degrees < 25) {
            cfg.usedTargetX = cfg.targetXRightPanel;
            cfg.usedTargetY = cfg.targetYRightPanel;
        }
        else if (degrees > 70) {
            cfg.usedTargetX = cfg.targetXLeftPanel;
            cfg.usedTargetY = cfg.targetYLeftPanel;
        }
        else {
            cfg.usedTargetX = cfg.targetXCenter;
            cfg.usedTargetY = cfg.targetYCenter;
        }

        RobotController.telemetry.addData("Calculated Rotation From Robot", degrees);
    }

    public static double calculateDistanceToWallInMeters(Pose robotPose, double targetX, double targetY) {
        return calculateDistance(robotPose, new Pose(targetX, targetY, 0), true);
    }

    Runnable stopMovingOuttakeGates = () -> {isMovingOuttakeGates = false;};
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose();
        return globalRobotPose;
    }
    
    protected ElapsedTime flashingTimer = new ElapsedTime();
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Blue);
    }
}
