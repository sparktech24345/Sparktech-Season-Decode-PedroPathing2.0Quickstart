package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Math;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Components.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.MAX_DISTANCE_MM;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.MAX_VOLTS;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.ballInIntakeThreshold;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceSensorName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalCamId;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftSensorColorMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.pose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.StateQueuer.MainQueuer;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.ActionSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexGamepad;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexTelemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.Configuration;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

@Config
@TeleOp(name = "Main TeleOp Blue", group = "AAA")
public class MainTeleOpBlue extends ComplexOpMode {
    protected VoltageSensor controlHubVoltageSensor;
    public static Pose farStart = pose(120, 24, 90); // no more reversing X
    public static double vp = 0.0055;//195;
    public static double vs = 0.1;//195;
    public static double velp = 180;
    public static double vd = 0;//25;
    public static double veld = 18;
    public static double vf = 0.0003;//15;
    public static double velf = 15;
    public static double vMultiplier = 0.9; /// TODO BE CAREFULL WITH THIS
    public static boolean shouldUseSecondaryPID = false;
    public static double targetVelocity = 0;

    public static Configuration cfg;

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
    public static boolean hasBallInIntake = false;
    public static boolean hasBallInRightChamber = false;
    public static boolean hasBallInLeftChamber = false;
    public static boolean shouldRemoveBalls = false;
    public static double OuttakePIDSwitch = 81;
    public static boolean shouldPullFromQueue = false;
    public static boolean wantsToTempOutputIntake = false;
    public static boolean shouldResetRightSensorBall = false;
    public static boolean ShouldSpewOutSensors = false;
    BallColorQueue ballColorQueue = new BallColorQueue();

    /// ----------------- Limelight Stuff -----------------
    protected Limelight3A limelight3A = null;
    public static double distanceMeasuredCamera = 0;
    public static double rotationDegreesMeasuredCamera = 0;
    public static double lastRotationDegreesMeasuredCamera = 0;
    public static boolean shouldShootOnCamera = false;
    public static double angleDecreaseValue = 10;
    public static double rightSideAngleBias = -2;
    public static double velocityDeltaCompensation = 80;
    public static int camId = 0;

    /// ----------------- Limelight Stuff -----------------
    private AnalogInput laserAnalog;

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

    /// ----------------- Outtake Priorities -----------------
    public static double turretAngleOverride = 0;
    public static double turretVelocityOverride = 0;

    public static double timer5 = 0;
    public static double timer6 = 0;
    public static double outtakeReversingTime = 180;
    public static double timer1 = 50; // far side
    public static double timer2 = 200; // far side
    public static double timer3 = 0; // close side
    public static double timer4 = 100; // close side
    public static double timer1ForSorting = 200 + 150;
    public static double timer2ForSorting = 300 + 150;
    public static double timer_far_v2 = 120;
    public static double timer_close_v2 = 120;
    public static double timerToFireBothFromTheLeft = 900; // close side
    public static double revUpTime = 1400;
    public static double timerToCloseGate = 200;
    public static double shootSortedTime = 800;
    public static double kVTurret = 0.003;
    public static double kATurret = 0.00015;
    public static double kSTurret = 0.06;
    public static double lookAheadSeconds = 0.45;
    public static double ballInAirTime = 1;
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

    /// ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Driver Stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

    public static boolean shouldShootWithoutTurret = false;
    public static boolean shouldForceOuttake = false;
    public static boolean wantsToFireWithIntakeUnsortedInSortingMode = false;
    public static double forcedOuttakeSpeed = 0.7;
    public static double oneTunnelDistance = 1.65;
/// ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Camera turret stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==
    public static double  maxiTurretAngle = 195;// de schimbat
    public static double cameraAngle;
    public static double miniTurretAngle = 165;//de schimbat
    public static double cameraModifier = 0;

    protected void robotMainLoop() {
        // all of the code

        // processing
        processCameraStuff();
        processTargetStuff(ComplexFollower.getCurrentPose(), cfg.targetX, cfg.targetY);
        distanceToWallOdometry = calculateDistanceToWallInMeters(ComplexFollower.getCurrentPose(), cfg.usedTargetX, cfg.usedTargetY);

        // Update pose from Odometry
        TurretRotateMotor.updateRobotPose(ComplexFollower.getCurrentPose());
        TurretRotateMotor.setBallTimeInAir(ballInAirTime);

        // this uses the processed target values
        //rotationToWallOdometry = calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), usedTargetX, cfg.usedTargetY);
        ComplexTelemetry.get().addData("distance to wall", distanceToWallOdometry);
        ComplexTelemetry.get().addData("fakeRotation", fakeRotation);
        ComplexTelemetry.get().addData("current cam id: ", camId);
        //colors
        handleColors();

        // Choosing which values to use
        double usedDistance = 0;
        double neededAngleForTurretRotation = 0;
        usedDistance = distanceToWallOdometry;

        rotationToWallOdometry = TurretRotateMotor.calculateLookaheadTarget(cfg.usedTargetX, cfg.usedTargetY, lookAheadSeconds);

        neededAngleForTurretRotation -= rotationToWallOdometry; /// TODO this might be to be reversed in some ways
        double camAngle = TurretRotateMotor.calculateLookaheadTarget_Camera(cfg.usedTargetX, cfg.usedTargetY);

        if (usedDistance > 2.9) neededAngleForTurretRotation += cfg.farZoneCameraAdder;
        if (shouldShootOnCamera) {
            if (Math.abs(TurretRotateMotor.getVelocity()) < 0.2)
                rotationAdder += rotationDegreesMeasuredCamera;

            neededAngleForTurretRotation += rotationAdder;
        }

        neededAngleForTurretRotation += D2_rotationAdder * D2_rotationAdderMulti;
        //if (neededAngleForTurretRotation > -30) neededAngleForTurretRotation += rightSideAngleBias;
        if (neededAngleForTurretRotation < 0) neededAngleForTurretRotation += 360;

        if (camAngle < miniTurretAngle) {
            cameraAngle = 180 - camAngle;
        } else if (camAngle > maxiTurretAngle) {
            cameraAngle = 180 + camAngle;
        } else {
            cameraAngle = 180;
        }

        ComplexTelemetry.get().addData("camera rotation target", camAngle);

        CameraRotateServo.setState(cameraAngle);


        ComplexTelemetry.get().addData("needed angle for turret rotation", neededAngleForTurretRotation);
        ComplexTelemetry.get().addData("angle for camera", cameraAngle);


        /// -= -= -= -= -= -= -= -= -= -= -= -= -= -= - Driver Buttons -= -= -= -= -= -= -= -= -= -= -= -= -= -= -

        // Special situations stuff
        if (ComplexGamepad.DPAD_LEFT1.get().pressed) {
            ComplexFollower.instance().setPose(pose(0, 0, 0));
            D2_velocityAdder = 0;
            D2_rotationAdder = 0;
        }
        shouldShootWithoutTurret = ComplexGamepad.DPAD_DOWN1.get().toggled;
        //shouldForceOuttake = ComplexGamepad.DPAD_UP1.get().toggled;

        if (ComplexGamepad.DPAD_RIGHT1.get().pressed || ComplexGamepad.RIGHT_BUMPER2.get().pressed) {
            MainQueuer.executeNow(new ActionSequence(// reverse for a bit
                    new GeneralAction(() -> wantsToTempOutputIntake = true),
                    new DelayAction(60),
                    new GeneralAction(() -> wantsToTempOutputIntake = false)
                    ));
        }



        // Driver Outputting
        if (ComplexGamepad.LEFT_BUMPER1.get().pressed) {
            wantsToOutput = !wantsToOutput;

            wantsToFireWithIntake = false;
            wantsToIntakeDriver = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        // Driver Intake
        if (ComplexGamepad.RIGHT_BUMPER1.get().pressed) {
            wantsToIntakeDriver = !wantsToIntakeDriver;

            wantsToFireWithIntake = false;
            wantsToOutput = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        // Driver Actually Shooting
        if (ComplexGamepad.RIGHT_TRIGGER1.get().pressed) {
            wantsToFireWithIntake = !wantsToFireWithIntake;
            hasBallInOuttake = false;
            hasJustBeganFiring = true;

            wantsToIntakeDriver = false;
            wantsToOutput = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        // Driver preparing for shooting
        if (ComplexGamepad.LEFT_TRIGGER1.get().pressed) isTryingToFire = !isTryingToFire;




        // Driver Switch Mode
        if (ComplexGamepad.Y1.get().pressed || ComplexGamepad.B2.get().pressed) {
            isInSortedMode = !isInSortedMode;
        }
        if (isInSortedMode) {
            gamepad1.setLedColor(0, 0, 255, 30000);
            gamepad2.setLedColor(0, 0, 255, 30000);
        }
        else {
            gamepad1.setLedColor(255, 0, 0, 30000);
            gamepad2.setLedColor(255, 0, 0, 30000);
        }
        // Driver fire unsorted in sorted mode
        if (ComplexGamepad.X1.get().pressed) {
            wantsToFireWithIntakeUnsortedInSortingMode = !wantsToFireWithIntakeUnsortedInSortingMode;
            hasJustBeganFiring = true;

            wantsToIntakeDriver = false;
            wantsToOutput = false;
            wantsToFireWithIntake = false;
            hasSwitchedIntakeState = true;
        }

        // Slowdown
        if (ComplexGamepad.B1.get().pressed) {
            if (ComplexGamepad.B1.get().toggled) {
                DriveTrain.setSlowdown(0.3);
            } else DriveTrain.setSlowdown(1);
        }

        // force store ball in outtake sorted if can, unsorted if cannot
        if (ComplexGamepad.A1.get().pressed) {
            hasBallInOuttake = false;
        }


        // ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Tilt Servo Stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  =
        // if (ComplexGamepad.DPAD_UP1.get().pressed) {
            // if (robot.getServoComponent("TiltServos").getPosition() == 1) // if retracted then extend
                // MainQueuer.executeNow(new StateAction(TiltServos", "EXTENDED"));
            // else // else retract back
                // MainQueuer.executeNow(new StateAction(TiltServos", "RETRACTED"));
        // }


        // ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Sorting Stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        // if (ComplexGamepad.RIGHT_BUMPER2.get().pressed) {
        //     ballColorQueue.add(BallColorSet_Decode.Purple);
        //     gamepad2.setLedColor(254, 0, 254, 1000000);
        // }
        // if (ComplexGamepad.LEFT_BUMPER2.get().pressed) {
        //     ballColorQueue.add(BallColorSet_Decode.Green);
        //     gamepad2.setLedColor(0, 254, 0, 1000000);
        // }
        // if (ComplexGamepad.B2.get().pressed) {
        //     ballColorQueue.clearQueue();
        //     gamepad2.setLedColor(254, 254, 254, 1000000);
        // }

        // Enable / Disable camera
        // Adders

        if (ComplexGamepad.DPAD_UP2.get().pressed) {
            D2_velocityAdder += 10;
        }

        if (ComplexGamepad.DPAD_DOWN2.get().pressed) {
            D2_velocityAdder -= 10;
        }

        if (ComplexGamepad.DPAD_DOWN2.get().held && ComplexGamepad.DPAD_UP2.get().held) {
            D2_velocityAdder = 0;
        }

        if (ComplexGamepad.DPAD_RIGHT2.get().pressed) {
            D2_rotationAdder += 1;
        }

        if (ComplexGamepad.DPAD_LEFT2.get().pressed) {
            D2_rotationAdder -= 1;
        }

        if (ComplexGamepad.DPAD_RIGHT2.get().held && ComplexGamepad.DPAD_LEFT2.get().held) {
            D2_rotationAdder = 0;
        }

        if (ComplexGamepad.RIGHT_TRIGGER2.get().held && ComplexGamepad.LEFT_BUMPER2.get().held) {
            ComplexFollower.instance().setPose(pose(cfg.hpResetX, cfg.hpResetY, cfg.hpResetDeg));
            D2_velocityAdder = 0;
            D2_rotationAdder = 0;
        }
        if (ComplexGamepad.LEFT_TRIGGER2.get().held && ComplexGamepad.LEFT_BUMPER2.get().held) {
            ComplexFollower.instance().setPose(pose(cfg.classifierResetX, cfg.classifierResetY, cfg.classifierResetDeg));
            D2_velocityAdder = 0;
            D2_rotationAdder = 0;
        }

        /// ==  ==  ==  ==  ==  ==  ==  ==  == Decision Making Code ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==


        // if switched intake states try turn off gates but can be overriden
        if (hasSwitchedIntakeState) {
            outtakeGatesState = -1;
            isMovingOuttakeGates = false;
            MainQueuer.executeNow(new StateAction(RightGateServo.states.CLOSED));
            MainQueuer.executeNow(new StateAction(LeftGateServo.states.CLOSED));
        }


        if (true) { // it includes the fire without sorting cuz one state at a time =  > it cant fail
            // UNSORTED MODE

            //just one channel logic
            if (wantsToIntakeDriver) {
                intakeGateState = 1; // always point to the right
                if (hasBallInRightChamber /*&& hasBallInOuttake*/) intakeGateState = -1; // should be 1 is -1 for temp
            }
            else intakeGateState = lastGateState;
            lastGateState = intakeGateState;



            if (wantsToIntakeDriver && false) {
                if (hasBallInRightChamber && !hasBallInOuttake) {
                    hasBallInOuttake = true;
                    isMovingOuttakeGates = true;
                    MainQueuer.executeNow(new ActionSequence(
                            new GeneralAction(() -> {
                                // reset remembering color logic
                                resetLeftBallColorTimer.reset();
                                shouldResetRightSensorBall = true;
                            }),
                            new StateAction(RightGateServo.states.OPEN),
                            new DelayAction(timerToCloseGate),
                            new StateAction(RightGateServo.states.CLOSED)
                    ));
                }
            }

            //shooting
            if (((wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) && hasJustBeganFiring) /*&& hasBallInLeftChamber*/) { // hasSwitchedIntakeState for do once logic
                isMovingOuttakeGates = true; // wont do special move commands until state is switched
                if (usedDistance > 2.9 /* && hasBallInLeftChamber*/ && !isInSortedMode) {
                    MainQueuer.executeNow(new ActionSequence(
                            new StateAction(RightGateServo.states.OPEN),
                            new DelayAction(timerToCloseGate), // 300mls
                            new StateAction(RightGateServo.states.CLOSED),
                            new DelayAction(timer1),
                            new StateAction(LeftGateServo.states.OPEN),
                            new DelayAction(timer2),
                            new StateAction(RightGateServo.states.OPEN)//                            new StateAction(LeftGateServo.states.OPEN),
//                            new DelayAction(timer_far_v2),
//                            new StateAction(RightGateServo.states.OPEN)
                    ));
                    hasJustBeganFiring = false;
                }
                else if (!isInSortedMode) {
                    MainQueuer.executeNow(new ActionSequence(
                            new StateAction(RightGateServo.states.OPEN),
                            new DelayAction(timerToCloseGate), // 300mls
                            new StateAction(RightGateServo.states.CLOSED),
                            new DelayAction(timer3),
                            new StateAction(LeftGateServo.states.OPEN),
                            new DelayAction(timer4),
                            new StateAction(RightGateServo.states.OPEN)
                            //new StateAction(LeftGateServo.states.OPEN),
                            //new DelayAction(timer_close_v2),
                            //new StateAction(RightGateServo.states.OPEN)

                    ));
                    hasJustBeganFiring = false;
                }
                else {
                    switch (camId) {
                        case 21 :
                            fireGPP();
                            break;
                        case 22:
                            firePGP();
                            break;
                        case 23:
                            firePPG();
                            break;
                    }
                    hasJustBeganFiring = false;
                }
            }

        }

        /// = -= -= -= -= -= end of big if (sorting) = -= -= -= -= -=

        // Closing gates if they arent needed
        if (!hasBallInLeftChamber && !hasBallInRightChamber) outtakeGatesState = -1;

        if (hasBallInLeftChamber && hasBallInRightChamber && hasBallInIntake)
            gamepad1.rumble(0.6, 0.6, 100);

        // IntakeStuff
        if (wantsToOutput) {
            intakeState = -1;
            isMovingOuttakeGates = false;
            outtakeGatesState = 1;
            if (hasBallInRightChamber) intakeGateState = 1; // open to the right so you can outtake
            else if (hasBallInLeftChamber) intakeGateState = -1; // left is lower priority but checked
        }
        else if (wantsToIntakeDriver) intakeState = 1;
        else if (wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) intakeState = 2;
        else intakeState = 0;

        if (wantsToTempOutputIntake)
            intakeState = -1;

        if ((wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) && Math.abs(TurretRotateMotor.getError()) > 15)
            intakeState = 0;

        hasSwitchedIntakeState = false;



        if ((ComplexGamepad.Y2.get().pressed)) camId = 21; // gpp
        if ((ComplexGamepad.X2.get().pressed)) camId = 22; // pgp
        if ((ComplexGamepad.A2.get().pressed)) camId = 23; // ppg

        switch (intakeState) {
            case -1:
                MainQueuer.executeNow(new StateAction(IntakeMotor.states.FULL_REVERSE));
                break;

            case 0:
                MainQueuer.executeNow(new StateAction(IntakeMotor.states.OFF));
                break;

            case 1:
                MainQueuer.executeNow(new StateAction(IntakeMotor.states.FULL));
                break;
            case 2:
                MainQueuer.executeNow(new StateAction(IntakeMotor.states.FIRING_POWER));
                break;
        }
        if (!isMovingOuttakeGates) {
            switch(outtakeGatesState) {
                case -1: // force close
                    MainQueuer.executeNow(new StateAction(RightGateServo.states.CLOSED));
                    MainQueuer.executeNow(new StateAction(LeftGateServo.states.CLOSED));
                    break;

                case 0:// do nothin
                    break;

                case 1: // force open
                    MainQueuer.executeNow(new StateAction(RightGateServo.states.OPEN));
                    MainQueuer.executeNow(new StateAction(LeftGateServo.states.OPEN));
                    break;
            }
        }

        shouldRemoveBalls = wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode || wantsToOutput;



        /// ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == End of logic code ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        // Outtake Stuff
        targetVelocity = 0;
        if (isTryingToFire) {
            // ----------------------- Power Stuff -----------------------

            targetVelocity = TurretRotateMotor.getTargetFlywheelVelocity(usedDistance) * vMultiplier + D2_velocityAdder;
            if (!shouldForceOuttake) {
                if (shouldUseSecondaryPID && Math.abs(targetVelocity - TurretSpinMotor.getVelocity()) <= OuttakePIDSwitch) {
                    TurretSpinMotor
                            .setOperationMode(MotorComponent.MotorModes.Velocity)
                            .setState((Math.eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                            .setVelocityCoefficients(velp, 0, veld, velf);
                }
                else {
                    TurretSpinMotor
                            .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                            .setState((Math.eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                            .setAccelerationVelocityCoefficients(vp, 0, vd, vf, vs);;
                }

            }
            else {
                TurretSpinMotor
                        .setOperationMode(MotorComponent.MotorModes.Power)
                        .setState(forcedOuttakeSpeed);
            }

            //if (Math.abs(TurretSpinMotor.getVelocity() - targetVelocity) <= 21)
            //    gamepad1.rumble(0, 0, 100);



                // ----------------------- Angle Stuff -----------------------
            double turretAngleVal = distanceToAngleFunction(usedDistance);
            turretAngleVal = clamp(turretAngleVal, 58, 295); // fresh measured
            TurretAngle.setState((Math.eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal));


            // ----------------------- Rotation Stuff -----------------------

            // Predict target (Target is at 0, 0 in world space for example)
            // lookaheadSeconds should roughly match your control loop latency + motor response time
            if (shouldShootWithoutTurret) neededAngleForTurretRotation = 0;
            TurretRotateMotor.setFeedforwardCoefficients(kVTurret, kATurret, kSTurret);
            TurretRotateMotor.setState(neededAngleForTurretRotation);

        }
        else {
            rotationAdder = 0;

            if (!shouldForceOuttake) {
                TurretSpinMotor
                        .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                        .setState(0);
            }
            else {
                TurretSpinMotor
                        .setOperationMode(MotorComponent.MotorModes.Power)
                        .setState(forcedOuttakeSpeed);
            }



            double turretAngleVal = distanceToAngleFunction(usedDistance);
            MainQueuer.executeNow(new StateAction(TurretAngle.states.DEFAULT));

            TurretRotateMotor.setState(0);
        }

        /// ==  ==  ==  ==  ==  ==  ==  ==  == Telemetry and Overrides ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==
        ComplexFollower.telemetry();
        ComplexTelemetry.get().addData("target vel", targetVelocity);
        ComplexTelemetry.get().addData("actual vel", TurretSpinMotor.getVelocity());
        ComplexTelemetry.get().addData("Intake Current", IntakeMotor.getCurrent());
    }


    // ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Init Stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==



    @Override
    public void initialize() {
        setStuffToDefault();

        DriveTrain.init();
        ComplexFollower.init(ConstantsDecode::createFollowerDecode);

        initOtherStuff(teamPipeline);
        makeConfig();

    }

    @Override
    public void update() {
        robotMainLoop();
    }

    @Override
    public void telemetry() {

    }

    @Override
    public void init_update() {
        super.init_update(); // remove this if you want to add stuff here
    }

    @Override
    public void on_start() {
        ComplexFollower.instance().setPose(globalRobotPose);
        camId = globalCamId;
    }

    @Override
    public void on_stop() {
        passPose();
    }

    public void setStuffToDefault() { // occasionally copy paste declarations here so we don't have surprises
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

        /// ----------------- Odometry Stuff -----------------
        distanceToWallOdometry = 0;
        rotationToWallOdometry = 0;
        rotationAdder = 0;
        lastRotationDegreesMeasuredCamera = 0;

        /// ----------------- Intake Priorities -----------------

        wantsToIntakeDriver = false;
        isInSortedMode = false;
        hasSwitchedIntakeState = false;
        wantsToFireWithIntake = false;
        wantsToOutput = false;
        isMovingOuttakeGates = false;
        hasJustBeganFiring = false;

        /// ----------------- Outtake Priorities -----------------
        turretAngleOverride = 0;
        turretVelocityOverride = 0;

        isTryingToFire = false;
        // ------------------ Driver 2 adders --------------------
        D2_velocityAdder = 0;
        D2_rotationAdder = 0;
    }

    public void initOtherStuff(int limelightPipeline) {
        //limelight stuff
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(limelightPipeline);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100); // poll 100 times per second
        limelight3A.start();

        //other stuff like color sensor
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        laserAnalog = hardwareMap.get(AnalogInput.class, distanceSensorName);

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        //gamepad1.setLedColor(254, 254, 254, 1000000);
        //gamepad2.setLedColor(254, 0, 0, 1000000);
    }


    // ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Color Stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

     protected void handleColors() {
        leftSensorColors = colorSensorLeft.getNormalizedColors();
        rightSensorColors = colorSensorRight.getNormalizedColors();

        Color.colorToHSV(leftSensorColors.toColor(), hsvLeftSensorColors);
        Color.colorToHSV(rightSensorColors.toColor(), hsvRightSensorColors);

        actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors, true);
        actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if (shouldResetRightSensorBall && resetLeftBallColorTimer.milliseconds() > 450) {
            shouldResetRightSensorBall = false;
            calculatedRightSensorDetectedBall = BallColorSet_Decode.NoBall;
        }



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


         /// distance sesnsor stuff

         // Read sensor voltage (0.0–3.3V)
         double volts = laserAnalog.getVoltage();
         // Convert voltage to distance in millimeters (linear mapping)
         double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
         hasBallInIntake = distanceMM < ballInIntakeThreshold;


         if (ShouldSpewOutSensors) {
            ComplexTelemetry.get().addData("LEFT_RED", (double)leftSensorColors.red * 10000.0 * leftSensorColorMultiplier);
            ComplexTelemetry.get().addData("LEFT_BLUE", (double)leftSensorColors.blue * 10000.0 * leftSensorColorMultiplier);
            ComplexTelemetry.get().addData("LEFT_GREEN", (double)leftSensorColors.green * 10000.0 * leftSensorColorMultiplier);

            ComplexTelemetry.get().addData("RIGHT_RED", (double)rightSensorColors.red * 10000.0);
            ComplexTelemetry.get().addData("RIGHT_BLUE", (double)rightSensorColors.blue * 10000.0);
            ComplexTelemetry.get().addData("RIGHT_GREEN", (double)rightSensorColors.green * 10000.0);

            ComplexTelemetry.get().addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
            ComplexTelemetry.get().addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
            ComplexTelemetry.get().addData("Has ball in intake", hasBallInIntake);
         }
    }


    // ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Camera Stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

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

        ComplexTelemetry.get().addData("areaPercentage", targetArea);
        double a = 8.60403612;
        double b = -0.0119936722;

        distanceMeasuredCamera = java.lang.Math.log(targetArea / a) / b;
    }

    // ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == Odometry Stuff ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

    public static double calculateHeadingAdjustment(Pose robotPose, double robotHeadingDeg, double targetX, double targetY) {
        // Current robot position
        double x = robotPose.getX();
        double y = robotPose.getY(); // don'// invert Y unless your coordinate system specifically requires it
        // Vector from robot to target
        double dx = targetX - x;
        double dy = targetY - y;

        // Angle from robot to target (in radians → degrees)
        double targetAngleDeg = java.lang.Math.toDegrees(java.lang.Math.atan2(dy, dx));

        // Normalize to [0, 360)
        if (targetAngleDeg < 0) targetAngleDeg += 360;

        // Robot heading in degrees
        // Assuming robotPose.getHeading() is in radians, 0° = facing +X, increases counterclockwise
        //double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Normalize to [0, 360)
        if (robotHeadingDeg < 0) robotHeadingDeg += 360;

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
        double angleRadians = java.lang.Math.atan2(dx, dy);

        return java.lang.Math.toDegrees(angleRadians) + 90;
    }

    public static void processTargetStuff(Pose pose, double targetX, double targetY) {
        double degrees = Math.abs(angleFromTargetToRobot(pose, targetX, targetY));

        if (degrees < 30) {
            // alpha is 1.0 at 0 degrees (Full Right) and 0.0 at 30 degrees (Center)
            double alpha = (30.0 - degrees) / 30.0;
            cfg.usedTargetX = lerp(cfg.targetXCenter, cfg.targetXRightPanel, alpha);
            cfg.usedTargetY = lerp(cfg.targetYCenter, cfg.targetYRightPanel, alpha);

        } else if (degrees > 65) {
            // alpha is 0.0 at 65 degrees (Center) and 1.0 at 90+ degrees (Full Left)
            // clamp alpha between 0 and 1 to prevent the target from sliding off the goal
            double alpha = Math.min(1.0, (degrees - 65.0) / 25.0);
            cfg.usedTargetX = lerp(cfg.targetXCenter, cfg.targetXLeftPanel, alpha);
            cfg.usedTargetY = lerp(cfg.targetYCenter, cfg.targetYLeftPanel, alpha);

        } else {
            //if between 30 and 65, stay locked on Center
            cfg.usedTargetX = cfg.targetXCenter;
            cfg.usedTargetY = cfg.targetYCenter;
        }

        ComplexTelemetry.get().addData("Calculated Rotation", degrees);
        ComplexTelemetry.get().addData("Target X", cfg.usedTargetX);
    }

    // Helper function for Linear Interpolation
    public static double lerp(double start, double end, double alpha) {
        return start + alpha * (end - start);
    }


    public static double calculateDistanceToWallInMeters(Pose robotPose, double targetX, double targetY) {
        return Math.calculateDistance(robotPose, new Pose(targetX, targetY, 0), true);
    }

    // ==  ==  = shooting sorted ==  ==  == /
    public void firePPG() {
        int greenBallPosition = 3;
        if (calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                MainQueuer.executeNow(new ActionSequence(// left right right
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(LeftGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 2: // if green is on the left
                MainQueuer.executeNow(new ActionSequence(// right right left
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 3: // green isnt or is in intake
                MainQueuer.executeNow(new ActionSequence(// right left right
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
        }
    }
    public void firePGP() {
        int greenBallPosition = 3;
        if (calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                MainQueuer.executeNow(new ActionSequence(// left right left
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(LeftGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 2: // if green is on the left
                MainQueuer.executeNow(new ActionSequence(// right left right
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 3: // green ball is in intake
                MainQueuer.executeNow(new ActionSequence(// right right left
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
        }
    }
    public void fireGPP() {
        int greenBallPosition = 3;
        if (calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if (calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                MainQueuer.executeNow(new ActionSequence(// right right left
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 2: // if green is on the left
                MainQueuer.executeNow(new ActionSequence(//left left right
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(LeftGateServo.states.CLOSED),
                        new DelayAction(timer1ForSorting),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer2ForSorting),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
            case 3: // green ball is in intake, cant actually sort this
                MainQueuer.executeNow(new ActionSequence(
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(timerToCloseGate),
                        new StateAction(RightGateServo.states.CLOSED),
                        new DelayAction(timer3),
                        new StateAction(LeftGateServo.states.OPEN),
                        new DelayAction(timer4),
                        new StateAction(RightGateServo.states.OPEN),
                        new DelayAction(1000),
                        new StateAction(RightGateServo.states.CLOSED),
                        new StateAction(LeftGateServo.states.CLOSED)
                ));
                break;
        }
    }


    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose(); //Math.toRadians
        return globalRobotPose;
    }

    protected ElapsedTime flashingTimer = new ElapsedTime();
    public void makeConfig() {
        cfg = Configuration.getConfig("blue");
    }
}
