package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.MAX_DISTANCE_MM;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.MAX_VOLTS;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.airSortingFunctionAngle;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.airSortingFunctionVelocity;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.ballInIntakeThreshold;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.calculateCameraAngle;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.calculateDistance;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.camOffsetX;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.convertCamAngleToServoValue;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceSensorName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalCamId;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftSensorColorMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.pose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.relocalizeRobot;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.timeToTurnAirSortOff;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.*;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.ActionSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.Drivers;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Config
@TeleOp(name="\uD83D\uDD35 Main TeleOp Blue", group="AAA") // 🔵
public class MainTeleOpBlue extends LinearOpMode {
    protected RobotController robot;
    //public static double targetVoltageForSpinMotors = 12.6;
    public static double vp = 0.001;//195;
    public static double vs = 0.18;//195;
    public static double velp = 180;
    public static double vd = 0;//25;
    public static double veld = 18;
    public static double vf = 0.00032; //0.0004
    public static double velf = 15;
    public static double vMultiplier = 1;
    public static boolean shouldUseSecondaryPID = false;
    public static int a;

    public static MainConfig cfg;
    public static Drivers driver = Drivers.Teo;

    /// ----------------- Target Pose Stuff ------------------

    public static double robotToGoalAbsoluteAngle = 0;

    /// ----------------- Color Sensor Stuff ------------------
    protected NormalizedColorSensor colorSensorRight;
    protected NormalizedColorSensor colorSensorLeft;
    protected NormalizedRGBA rightSensorColors;
    protected NormalizedRGBA leftSensorColors;
    // final float[] hsvRightSensorColors = new float[3];
    // final float[] hsvLeftSensorColors = new float[3];
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
    public static boolean disableTurret = false;
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
    public static double flywheelVelocity = 0;
    public static double lastFlywheelVelocity = 0;
    public static boolean shouldToAirSort = false;
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
    public static double turretAimOverride = 0;

    public static double timer5 = 0;
    public static double timer6 = 0;
    public static double outtakeReversingTime = 180;
    public static double timer1 = 10; // smol
    public static double timer2 = 320; // far side
    public static double timer3 = 0; // close side
    public static double timer4 = 320; // close side
    public static double timer1ForSorting = 200 + 150;
    public static double timer2ForSorting = 300 + 150;
    public static double mainTimerForSorting = 280;
    public static double timerBothOnOneChannelTimerForSorting = 600;
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
    public static double cameraImaginaryX = 110;
    public static Pose farPark = pose(117, 12, 90);

    public static double D2_velocityAdder = 0;
    public static double D2_rotationAdder = 0;

    public static double D2_velocityAdderMulti = 1;
    public static double D2_rotationAdderMulti = 1;
    public static double batteryToFireThreshold = 50;

    /// ================================== Driver Stuff  ==================================

    public static boolean shouldShootWithoutTurret = false;
    public static boolean shouldForceOuttake = false;
    public static boolean wantsToFireWithIntakeUnsortedInSortingMode = false;
    public static double forcedOuttakeSpeed = 0.7;
    public static double oneTunnelDistance = 1.65;
///==============================Camera turret stuff============================================
    public static double  maxiTurretAngle = 195;// de schimbat
    public static double cameraAngle=0;
    public static double cameraAngleOverite=176;
    public static double miniTurretAngle = 165;//de schimbat
    public static double cameraModifier = 0;

    protected void robotMainLoop() {
        // all the code

        // processing
        processTargetStuff(robot.getCurrentPose(), cfg.targetX, cfg.targetY);
        distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.usedTargetX, cfg.usedTargetY);

        TurretComponent tempTurret = robot.getTurretComponent("TurretRotateMotor");
        // Update pose from Odometry
        tempTurret.updateRobotPose(robot.getCurrentPose());
        tempTurret.setBallTimeInAir(ballInAirTime);

        // this uses the processed target values
        //rotationToWallOdometry = calculateHeadingAdjustment(robot.getCurrentPose(), Math.toDegrees(robot.getCurrentPose().getHeading()), usedTargetX, cfg.usedTargetY);
        RobotController.telemetry.addData("distance to wall", distanceToWallOdometry);
        RobotController.telemetry.addData("fakeRotation", fakeRotation);
        RobotController.telemetry.addData("current cam id: ", camId);
        //colors
        handleColors();

        // Choosing which values to use
        double usedDistance = 0;
        double neededAngleForTurretRotation = 0;
        usedDistance = distanceToWallOdometry;

        rotationToWallOdometry = tempTurret.calculateLookaheadTarget(usedTargetX, usedTargetY, lookAheadSeconds);

        neededAngleForTurretRotation -= rotationToWallOdometry; /// TODO this might be to be reversed in some ways


        if (usedDistance > 2.9) neededAngleForTurretRotation += cfg.farZoneCameraAdder;
        if (shouldShootOnCamera) {
            if (Math.abs(robot.getMotorComponent("TurretRotateMotor").getVelocity()) < 0.2)
                rotationAdder += rotationDegreesMeasuredCamera;

            neededAngleForTurretRotation += rotationAdder;
        }

        neededAngleForTurretRotation += D2_rotationAdder * D2_rotationAdderMulti;
        //if(neededAngleForTurretRotation > -30) neededAngleForTurretRotation += rightSideAngleBias;
        if (neededAngleForTurretRotation < 0) neededAngleForTurretRotation += 360;

        double camAngle = - calculateCameraAngle(camTargetX,camTargetY,robot.getCurrentPose(),camOffsetX,0);
        double middleOfTheFieldY =0;
        if(currentTeamColor == TeamColor.Blue) middleOfTheFieldY = -16; else middleOfTheFieldY = 16;
        if(robotToGoalAbsoluteAngle > 20 && robotToGoalAbsoluteAngle < 38 && usedDistance > 3.4) // the params for the small triangle
            camAngle = - calculateCameraAngle(cameraImaginaryX,middleOfTheFieldY,robot.getCurrentPose(),camOffsetX,0);
        cameraAngle = convertCamAngleToServoValue(camAngle);
        cameraAngle = clamp(cameraAngle,0,360); // de notat ca are range de 310 grade defapt

        RobotController.telemetry.addData("camera rotation target", camAngle);
        robot.getServoComponent("CameraRotateServo")
                .setTarget((eval(cameraAngleOverite) ? cameraAngleOverite : cameraAngle));


        RobotController.telemetry.addData("needed angle for turret rotation", neededAngleForTurretRotation);
        RobotController.telemetry.addData("TurretRotate power", tempTurret.getPower());
        RobotController.telemetry.addData("angle for camera", cameraAngle);


        processCameraStuff(cameraAngle); // pass the thing that the servo gets


        /// -=-=-=-=-=-=-=-=-=-=-=-=-=-=- Driver Buttons -=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        // Special situations stuff
        if (robot.getKey("DPAD_LEFT1").ExecuteOnPress) {
            ComplexFollower.instance().setPose(pose(0, 0, 0));
            D2_velocityAdder = 0;
            D2_rotationAdder = 0;
        }
        shouldShootWithoutTurret = robot.getKey("DPAD_DOWN1").IsToggledOnPress;
        //shouldForceOuttake = robot.getKey("DPAD_UP1").IsToggledOnPress;

        if(robot.getKey("DPAD_RIGHT1").ExecuteOnPress || robot.getKey("RIGHT_BUMPER2").ExecuteOnPress){
            robot.executeNow(new ActionSequence( // reverse for a bit
                    new GeneralAction(() -> wantsToTempOutputIntake = true),
                    new DelayAction(45),
                    new GeneralAction(() -> wantsToTempOutputIntake = false)
                    ));
        }

        if(robot.getKey("LEFT_STICK_BUTTON1").ExecuteOnPress) disableTurret = !disableTurret;



        // Driver Outputting
        if (robot.getKey("LEFT_BUMPER1").ExecuteOnPress){
            wantsToOutput = !wantsToOutput;

            wantsToFireWithIntake = false;
            wantsToIntakeDriver = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        // Driver Intake
        if (robot.getKey("RIGHT_BUMPER1").ExecuteOnPress){
            wantsToIntakeDriver = !wantsToIntakeDriver;

            wantsToFireWithIntake = false;
            wantsToOutput = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        // Driver Actually Shooting
        if (robot.getKey("RIGHT_TRIGGER1").ExecuteOnPress){
            wantsToFireWithIntake = !wantsToFireWithIntake;
            hasBallInOuttake = false;
            hasJustBeganFiring = true;

            wantsToIntakeDriver = false;
            wantsToOutput = false;
            wantsToFireWithIntakeUnsortedInSortingMode = false;
            hasSwitchedIntakeState = true;
        }
        // Driver preparing for shooting
        if(robot.getKey("LEFT_TRIGGER1").ExecuteOnPress) isTryingToFire = !isTryingToFire;




        // Driver Switch Mode
        if (robot.getKey("Y1").ExecuteOnPress || robot.getKey("B2").ExecuteOnPress){
            isInSortedMode = !isInSortedMode;
        }
        if(isInSortedMode){
            gamepad1.setLedColor(0,0,255,30000);
            gamepad2.setLedColor(0,0,255,30000);
        }
        else{
            gamepad1.setLedColor(255,0,0,30000);
            gamepad2.setLedColor(255,0,0,30000);
        }
        // Driver fire unsorted in sorted mode
        if (robot.getKey("X1").ExecuteOnPress){
            wantsToFireWithIntakeUnsortedInSortingMode = !wantsToFireWithIntakeUnsortedInSortingMode;
            hasJustBeganFiring = true;

            wantsToIntakeDriver = false;
            wantsToOutput = false;
            wantsToFireWithIntake = false;
            hasSwitchedIntakeState = true;
        }

        // Slowdown
        if (robot.getKey("B1").ExecuteOnPress) {
            if (robot.getKey("B1").IsToggledOnPress) {
                DriveTrain.setSlowdown(0.3);
            } else DriveTrain.setSlowdown(1);
        }

        // force store ball in outtake sorted if can, unsorted if cannot
        if (robot.getKey("A1").ExecuteOnPress) {
            hasBallInOuttake = false;
        }


        // ====================== Tilt Servo Stuff ===================
        if(robot.getKey("DPAD_UP1").ExecuteOnPress){
            if(robot.getServoComponent("TiltServos").getPosition() == 0.85) // if retracted then extend
                robot.executeNow(new StateAction("TiltServos","EXTENDED"));
            else // else retract back
                robot.executeNow(new StateAction("TiltServos","RETRACTED"));
        }

        // Adders

        if (robot.getKey("DPAD_UP2").ExecuteOnPress) {
            D2_velocityAdder += 10;
        }

        if (robot.getKey("DPAD_DOWN2").ExecuteOnPress) {
            D2_velocityAdder -= 10;
        }

//        if (robot.getKey("DPAD_DOWN2").IsHeld && robot.getKey("DPAD_UP2").IsHeld) {
//            D2_velocityAdder = 0;
//        }

        if (robot.getKey("DPAD_RIGHT2").ExecuteOnPress) {
            D2_rotationAdder += 1;
        }

        if (robot.getKey("DPAD_LEFT2").ExecuteOnPress) {
            D2_rotationAdder -= 1;
        }

//        if (robot.getKey("DPAD_RIGHT2").IsHeld && robot.getKey("DPAD_LEFT2").IsHeld) {
//            D2_rotationAdder = 0;
//        }

        if(robot.getKey("RIGHT_TRIGGER2").IsHeld && robot.getKey("LEFT_BUMPER2").IsHeld) {
            ComplexFollower.instance().setPose(pose(cfg.hpResetX, cfg.hpResetY, cfg.hpResetDeg));
            D2_velocityAdder = 0;
            D2_rotationAdder = 0;
        }
        if(robot.getKey("LEFT_TRIGGER2").IsHeld && robot.getKey("LEFT_BUMPER2").IsHeld) {
            ComplexFollower.instance().setPose(pose(cfg.classifierResetX,cfg.classifierResetY,cfg.classifierResetDeg));
            D2_velocityAdder = 0;
            D2_rotationAdder = 0;
        }

        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Decision Making Code ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==


        // if switched intake states try turn off gates but can be overriden
        if(hasSwitchedIntakeState){
            outtakeGatesState = -1;
            isMovingOuttakeGates = false;
            robot.executeNow(new StateAction("RightGateServo", "CLOSED"));
            robot.executeNow(new StateAction("LeftGateServo", "CLOSED"));
        }


        if(true){ // it includes the fire without sorting cuz one state at a time => it cant fail
            // UNSORTED MODE

            //just one channel logic
            if(wantsToIntakeDriver){
                intakeGateState = 1; // always point to the right
                if(hasBallInRightChamber /*&& hasBallInOuttake*/) intakeGateState = -1; // should be 1 is -1 for temp
            }
            else intakeGateState = lastGateState;
            lastGateState = intakeGateState;



            if(wantsToIntakeDriver && false){
                if(hasBallInRightChamber && !hasBallInOuttake){
                    hasBallInOuttake = true;
                    isMovingOuttakeGates = true;
                    robot.executeNow(new ActionSequence(
                            new GeneralAction(() -> {
                                // reset remembering color logic
                                resetLeftBallColorTimer.reset();
                                shouldResetRightSensorBall = true;
                            }),
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate),
                            new StateAction("RightGateServo", "CLOSED")
                    ));
                }
            }

            if(!wantsToFireWithIntake && camId == 21 && calculatedLeftSensorDetectedBall == BallColorSet_Decode.Purple && calculatedRightSensorDetectedBall == BallColorSet_Decode.Purple)
                shouldToAirSort = true;

            if(camId != 21) shouldToAirSort = false;
            if(!wantsToFireWithIntake && (calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green || calculatedRightSensorDetectedBall == BallColorSet_Decode.Green))
                shouldToAirSort = false;
            if(!isInSortedMode)
                shouldToAirSort = false;


            //shooting
            if(((wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) && hasJustBeganFiring) /*&& hasBallInLeftChamber*/){ // hasSwitchedIntakeState for do once logic
                isMovingOuttakeGates = true; // wont do special move commands until state is switched
                if (usedDistance > 2.9 /* && hasBallInLeftChamber*/ && !isInSortedMode) {
                    robot.executeNow(new ActionSequence(
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate), // 300mls
                            new StateAction("RightGateServo", "CLOSED"),
                            //new DelayAction(timer1),
                            new StateAction("LeftGateServo", "OPEN"),
                            new DelayAction(timer2),
                            new StateAction("RightGateServo", "OPEN")//                            new StateAction("LeftGateServo", "OPEN"),
//                            new StateAction("LeftGateServo", "OPEN"),
//                            new DelayAction(timer_far_v2),
//                            new StateAction("RightGateServo", "OPEN")
                    ));
                    hasJustBeganFiring = false;
                }
                else if(!isInSortedMode){
                    robot.executeNow(new ActionSequence(
                            new StateAction("RightGateServo", "OPEN"),
                            new DelayAction(timerToCloseGate), // 300mls
                            new StateAction("RightGateServo", "CLOSED"),
                            //new DelayAction(timer1),
                            new StateAction("LeftGateServo", "OPEN"),
                            new DelayAction(timer4),
                            new StateAction("RightGateServo", "OPEN")
//                            new StateAction("LeftGateServo", "OPEN"),
//                            new DelayAction(timer_close_v2),
//                            new StateAction("RightGateServo", "OPEN")
//
                    ));
                    hasJustBeganFiring = false;
                }
                else {
                    switch (camId){
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

        /// =-=-=-=-=-=  end of big if(sorting)  =-=-=-=-=-=

        // Closing gates if they arent needed
        if(!hasBallInLeftChamber && !hasBallInRightChamber) outtakeGatesState = -1;

//        if(hasBallInLeftChamber && hasBallInRightChamber && hasBallInIntake)
//            gamepad1.rumble(0.6,0.6,100);

        // IntakeStuff
        if(wantsToOutput){
            intakeState = -1;
            isMovingOuttakeGates = false;
            outtakeGatesState = 1;
            if (hasBallInRightChamber) intakeGateState = 1; // open to the right so you can outtake
            else if (hasBallInLeftChamber) intakeGateState = -1; // left is lower priority but checked
        }
        else if(wantsToIntakeDriver) intakeState = 1;
        else if(wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) intakeState = 2;
        else intakeState = 0;

        if(wantsToTempOutputIntake)
            intakeState = 0;

        if((wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) && Math.abs(tempTurret.getError()) > 15)
            intakeState = 0;

        double fakeVelocity = distanceToVelocityFunction(usedDistance) * vMultiplier + D2_velocityAdder;
        if((wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode) && RobotController.currentVoltage < 10 && fakeVelocity - robot.getMotorComponent("TurretSpinMotor").getVelocity()>= batteryToFireThreshold)
            intakeState = 0;
        RobotController.telemetry.addData("velocity difrence",fakeVelocity - robot.getMotorComponent("TurretSpinMotor").getVelocity()>= batteryToFireThreshold);

        hasSwitchedIntakeState = false;



        if ((robot.getKey("Y2").ExecuteOnPress )) camId = 21; // gpp
        if ((robot.getKey("X2").ExecuteOnPress)) camId = 22; // pgp
        if ((robot.getKey("A2").ExecuteOnPress)) camId = 23; // ppg

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
        if(!isMovingOuttakeGates){
            switch(outtakeGatesState){
                case -1: // force close
                    robot.executeNow(new StateAction("RightGateServo", "CLOSED"));
                    robot.executeNow(new StateAction("LeftGateServo", "CLOSED"));
                    break;

                case 0:// do nothin
                    break;

                case 1: // force open
                    robot.executeNow(new StateAction("RightGateServo", "OPEN"));
                    robot.executeNow(new StateAction("LeftGateServo", "OPEN"));
                    break;
            }
        }

        shouldRemoveBalls = wantsToFireWithIntake || wantsToFireWithIntakeUnsortedInSortingMode || wantsToOutput;



        /// ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == End of logic code ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        // Outtake Stuff
        double targetVelocity = 0;
        if (isTryingToFire) {
            // ----------------------- Power Stuff -----------------------

            targetVelocity = distanceToVelocityFunction(usedDistance) * vMultiplier + D2_velocityAdder;
            if(RobotController.currentVoltage < 10 || RobotController.getDrivetrainCumulativePower() >3.85) targetVelocity += 30;
            if(shouldToAirSort) targetVelocity = airSortingFunctionVelocity(usedDistance) *vMultiplier + D2_velocityAdder;
            if(!shouldForceOuttake){
                if(shouldUseSecondaryPID /*always false */ && Math.abs(targetVelocity - robot.getMotorComponent("TurretSpinMotor").getVelocity()) <= OuttakePIDSwitch){
                    robot.getMotorComponent("TurretSpinMotor")
                            .setOperationMode(MotorComponent.MotorModes.Velocity)
                            .setTarget((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                            .setVelocityCoefficients(velp,0,veld,velf);
                }
                else {
                    robot.getMotorComponent("TurretSpinMotor")
                            .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                            .setTarget((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                            .setAccelerationVelocityCoefficients(vp,0,vd,vf,vs)
//                            .setVoltageCompensation(true)
//                            .setTargetVoltage(targetVoltageForSpinMotors)
                    ;
                }

            }
            else {
                robot.getMotorComponent("TurretSpinMotor")
                        .setOperationMode(MotorComponent.MotorModes.Power)
                        .setTarget(forcedOuttakeSpeed);
            }

            if(Math.abs(robot.getMotorComponent("TurretSpinMotor").getVelocity() - targetVelocity) <= 21)
                gamepad1.rumble(0.4,0.4,100);



                // ----------------------- Angle Stuff -----------------------
            double turretAngleVal = 0;
            turretAngleVal = distanceToAngleFunction(usedDistance);
            if(shouldToAirSort) turretAngleVal = airSortingFunctionAngle(usedDistance);
            RobotController.telemetry.addData("angle function output",turretAngleVal);
            turretAngleVal = (eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal);
            turretAngleVal = clamp(turretAngleVal,18, 324); // fresh measured
            robot.getServoComponent("TurretAngle")
                    .setTarget(turretAngleVal);


            // ----------------------- Rotation Stuff -----------------------

            // Predict target (Target is at 0,0 in world space for example)
            // lookaheadSeconds should roughly match your control loop latency + motor response time
            if(shouldShootWithoutTurret) neededAngleForTurretRotation = 0;
            tempTurret.setFeedforwardCoefficients(kVTurret,kATurret,kSTurret);
            tempTurret.setTarget(neededAngleForTurretRotation);

        }
        else {
            rotationAdder = 0;

            if(!shouldForceOuttake){
                robot.getMotorComponent("TurretSpinMotor")
                        .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                        .setTarget(0);
            }
            else{
                robot.getMotorComponent("TurretSpinMotor")
                        .setOperationMode(MotorComponent.MotorModes.Power)
                        .setTarget(forcedOuttakeSpeed);
            }



            // double turretAngleVal = distanceToAngleFunction(usedDistance);
            // TODO: check why tf was this here ^
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT"));

            robot.getMotorComponent("TurretRotateMotor")
                    .setTarget(0);


        }

        // checking if it has fired a ball
        lastFlywheelVelocity = flywheelVelocity;
        flywheelVelocity = robot.getMotorComponent("TurretSpinMotor").getVelocity();
        if(wantsToFireWithIntake && lastFlywheelVelocity - flywheelVelocity >= 100){
            //ballCounter++;
            //shouldToAirSort = false; wayy too litle too late
        }


        if(disableTurret){
            tempTurret.setOperationMode(MotorComponent.MotorModes.Power).setTarget(0);
            robot.getMotorComponent("TurretSpinMotor").setOperationMode(MotorComponent.MotorModes.Power).setTarget(0);
        }

        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Telemetry and Overrides ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==
        robot.spitFollowerTelemetry();
        RobotController.telemetry.addData("target vel",targetVelocity);
        RobotController.telemetry.addData("actual vel",flywheelVelocity);
        RobotController.telemetry.addData("actual pow",robot.getMotorComponent("TurretSpinMotor").getPower());
        RobotController.telemetry.addData("Intake Current",robot.getMotorComponent("IntakeMotor").getCurrent());
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
        initOtherStuff(teamPipeline);
        robot.UseDefaultMovement();
        makeConfig();

        while (opModeInInit()) {
            robot.init_loop();
        }
        ComplexFollower.instance().setPose(globalRobotPose);
        camId = globalCamId;

        while (opModeIsActive()) {
            // loop
            robot.loop();
        }
        passPose();
    }

    public void setStuffToDefault(){ // occasionally copy and paste declarations here so we don't have surprises
        ballCounter = 0;
        shouldToAirSort = false;
        shouldShootOnCamera = false;
        shouldRemoveBalls = false;
        shouldPullFromQueue = false;
        hasBallInOuttake = false;
        wantsToTempOutputIntake = false;
        shouldResetRightSensorBall = false;
        disableTurret = false;
        if (ballColorQueue == null) ballColorQueue = new BallColorQueue(); robotToGoalAbsoluteAngle = 0;
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
        limelight3A.pipelineSwitch(1); // relocalization
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100); // poll 100 times per second
        limelight3A.start();

        //other stuff like color sensor
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        laserAnalog = hardwareMap.get(AnalogInput.class, distanceSensorName);

        //gamepad1.setLedColor(254, 254, 254, 1000000);
        //gamepad2.setLedColor(254, 0, 0, 1000000);
    }


    // ============================ Color Stuff ============================

     protected void handleColors() {

         leftSensorColors = colorSensorLeft.getNormalizedColors();
         rightSensorColors = colorSensorRight.getNormalizedColors();

         //Color.colorToHSV(leftSensorColors.toColor(), hsvLeftSensorColors);
         //Color.colorToHSV(rightSensorColors.toColor(), hsvRightSensorColors);
         // TODO: check why these 2 commented lines were here before since the hsv values arent used

         actualLeftSensorDetectedBall = BallColorSet_Decode.getColorForStorage(leftSensorColors, true);
         actualRightSensorDetectedBall = BallColorSet_Decode.getColorForStorage(rightSensorColors);

        if(shouldResetRightSensorBall && resetLeftBallColorTimer.milliseconds() > 450) {
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


         if(ShouldSpewOutSensors) {
             RobotController.telemetry.addData("LEFT_RED", (double)leftSensorColors.red * 10000.0 * leftSensorColorMultiplier);
             RobotController.telemetry.addData("LEFT_BLUE", (double)leftSensorColors.blue * 10000.0 * leftSensorColorMultiplier);
             RobotController.telemetry.addData("LEFT_GREEN", (double)leftSensorColors.green * 10000.0 * leftSensorColorMultiplier);

             RobotController.telemetry.addData("RIGHT_RED", (double)rightSensorColors.red * 10000.0);
             RobotController.telemetry.addData("RIGHT_BLUE", (double)rightSensorColors.blue * 10000.0);
             RobotController.telemetry.addData("RIGHT_GREEN", (double)rightSensorColors.green * 10000.0);

             RobotController.telemetry.addData("LEFT Sensed Color", calculatedLeftSensorDetectedBall);
             RobotController.telemetry.addData("RIGHT Sensed Color", calculatedRightSensorDetectedBall);
             RobotController.telemetry.addData("Has ball in intake", hasBallInIntake);
         }
    }


    // ============================ Camera Stuff ============================

    public void processCameraStuff(double camangle) {
        LLResult result = limelight3A.getLatestResult();
        RobotController.telemetry.addData("Is Valid",result.isValid());
        RobotController.telemetry.addData("Robot Pinpoint Yaw", Math.toDegrees(ComplexFollower.getFollowerInstance().getHeading()));

        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                RobotController.telemetry.addData("tx", result.getTx());
                RobotController.telemetry.addData("ty", result.getTy());
                RobotController.telemetry.addData("Number of Tags Seen", result.getBotposeTagCount());
                RobotController.telemetry.addData("Botpose", botpose.toString());
                RobotController.telemetry.addData("Botpose Yaw Smth", botpose.getOrientation());
                RobotController.telemetry.addData("Botpose X", botpose.getPosition().x);
                RobotController.telemetry.addData("Botpose Y", botpose.getPosition().y);

                Pose tempPose = relocalizeRobot(botpose.getPosition().x,botpose.getPosition().y,botpose.getOrientation().getYaw(AngleUnit.DEGREES),
                        currentTeamColor,camOffsetX,0,
                        camangle,result.getBotposeTagCount());

                RobotController.telemetry.addData("Bots Wanna be Position",tempPose.toString());



            }
        }
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
        robotToGoalAbsoluteAngle = Math.abs(angleFromTargetToRobot(pose, targetX, targetY));

        if (robotToGoalAbsoluteAngle < 30) {
            // alpha is 1.0 at 0 degrees (Full Right) and 0.0 at 30 degrees (Center)
            double alpha = (30.0 - robotToGoalAbsoluteAngle) / 30.0;
            cfg.usedTargetX = lerp(cfg.targetXCenter, cfg.targetXRightPanel, alpha);
            cfg.usedTargetY = lerp(cfg.targetYCenter, cfg.targetYRightPanel, alpha);

        } else if (robotToGoalAbsoluteAngle > 65) {
            // alpha is 0.0 at 65 degrees (Center) and 1.0 at 90+ degrees (Full Left)
            // clamp alpha between 0 and 1 to prevent the target from sliding off the goal
            double alpha = Math.min(1.0, (robotToGoalAbsoluteAngle - 65.0) / 25.0);
            cfg.usedTargetX = lerp(cfg.targetXCenter, cfg.targetXLeftPanel, alpha);
            cfg.usedTargetY = lerp(cfg.targetYCenter, cfg.targetYLeftPanel, alpha);

        } else {
            //if between 30 and 65, stay locked on Center
            cfg.usedTargetX = cfg.targetXCenter;
            cfg.usedTargetY = cfg.targetYCenter;
        }

        RobotController.telemetry.addData("Calculated Rotation", robotToGoalAbsoluteAngle);
        RobotController.telemetry.addData("Target X", cfg.usedTargetX);
    }

    // Helper function for Linear Interpolation
    public static double lerp(double start, double end, double alpha) {
        return start + alpha * (end - start);
    }


    public static double calculateDistanceToWallInMeters(Pose robotPose, double targetX, double targetY) {
        return calculateDistance(robotPose, new Pose(targetX, targetY, 0), true);
    }

    //===== shooting sorted ======/
    public void firePPG(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // left right right
                        new StateAction("LeftGateServo", "OPEN"), // left left
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence( // right right left
                        new StateAction("RightGateServo", "OPEN"), // right right
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green isnt or is in intake
                robot.executeNow(new ActionSequence( // right left right
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        //new DelayAction(timer1),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(mainTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
        }
    }
    /*
                            */
    public void firePGP(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // left right left
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(timerToCloseGate),
                        new StateAction("LeftGateServo", "CLOSED"),
                        //new DelayAction(timer1),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(mainTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence(// right left right
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        //new DelayAction(timer1),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(mainTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green ball is in intake
                robot.executeNow(new ActionSequence( // right right left1000
                        new StateAction("RightGateServo", "OPEN"), // right right
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
        }
    }
    public void fireGPP(){
        int greenBallPosition = 3;
        if(calculatedRightSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 1; // green is on the right
        else if(calculatedLeftSensorDetectedBall == BallColorSet_Decode.Green) greenBallPosition = 2; // green is on the left
        else greenBallPosition = 3; // green is on the right
        switch (greenBallPosition) {
            case 1: // green on the right
                robot.executeNow(new ActionSequence( // right left right
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(timerToCloseGate),
                        new StateAction("RightGateServo", "CLOSED"),
                        //new DelayAction(timer1),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(mainTimerForSorting),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 2: // if green is on the left
                robot.executeNow(new ActionSequence( //left right left
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(timerToCloseGate),
                        new StateAction("LeftGateServo", "CLOSED"),
                        //new DelayAction(timer1),
                        new StateAction("RightGateServo", "OPEN"), // right
                        new DelayAction(mainTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
            case 3: // green ball is in intake, cant actually sort this airsort needed
                robot.executeNow(new ActionSequence( // it willl fire right right left and try air sort
                        new StateAction("RightGateServo", "OPEN"), // right right
                        new GeneralAction(turnAirSortOff),
                        new DelayAction(timerBothOnOneChannelTimerForSorting),
                        new StateAction("LeftGateServo", "OPEN"), // left
                        new DelayAction(600),
                        new StateAction("RightGateServo", "CLOSED"),
                        new StateAction("LeftGateServo", "CLOSED")
                ));
                break;
        }
    }


    // ============================ Weird Stuff ============================
    Runnable stopMovingOuttakeGates = () -> {isMovingOuttakeGates = false;};
    Runnable turnAirSortOff = () -> {
        robot.executeNow(new ActionSequence(
                new DelayAction(timeToTurnAirSortOff),
                new GeneralAction(() -> shouldToAirSort = false)
        ));
    };
    public Pose passPose() {
        globalRobotPose = ComplexFollower.instance().getPose(); //Math.toRadians
        return globalRobotPose;
    }
    
    protected ElapsedTime flashingTimer = new ElapsedTime();
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Blue);
    }
}
