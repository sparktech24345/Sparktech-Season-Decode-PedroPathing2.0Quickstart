package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.calculateDistance;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.degreesToOuttakeTurretServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.greenSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.launchSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall1;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

@Config
@TeleOp(name="Main TeleOP Blue", group="Main")
public class MainTeleOPBlue extends LinearOpMode {
    protected RobotController robot;
    protected VoltageSensor controlHubVoltageSensor;
    public static double targetX = 125;
    public static double targetY = 46;
    public static Pose farStart = new Pose(120, 24, Math.toRadians(90)); // no more reversing X

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
    public static double distanceMeasuredCamera =0;
    public static double rotationDegreesMeasuredCamera =0;
    public static double cameraErrorMultiplier = 1;
    public static double encoderMultiplier = 1;
    public static double cameraAdder = 0;
    public static double farZoneCameraAdder = -2;
    public static boolean shouldShootOnCamera = false;

    /// ----------------- Limelight Stuff -----------------
    private AnalogInput laserAnalog; // if < 110 then has ball
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    /// ----------------- Odometry Stuff -----------------
    public static double distanceToWallOdometry = 0;
    public static double rotationToWallOdometry = 0;

    /// ----------------- Intake Priorities -----------------
    protected ElapsedTime sortingShootTimer = new ElapsedTime();
    public static boolean wantsToIntakeDriver = false;
    public static boolean wantsToFireWithIntake = false;
    public static boolean wantsToFireSortingWithIntake = false;
    public static boolean wantsToOutput = false;
    public static int gateState = 0;

    /// ----------------- Outtake Priorities -----------------
    public static double turretAngleOverride = 0;
    public static double turretVelocityOverride = 0;
    public static double timer1 = 600;
    public static double timer2 = 600;
    public static double timer3 = 500;

    public static boolean isTryingToFire = false;
    public static boolean needsToLowerGates = true;
    protected void robotMainLoop() {
        // all of the code

        // processing
        processCameraStuff();
        distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), targetX, targetY);
        rotationToWallOdometry = calculateHeadingAdjustment(robot.getCurrentPose(), targetX, targetY);
        RobotController.telemetry.addData("distance to wall", distanceToWallOdometry);

        //if(rotationDegreesMeasuredCamera < 5 && rotationDegreesMeasuredCamera != 0) gamepad1.setLedColor(0,254,0,250);
        //else gamepad1.setLedColor(254,0,0,250);


        //colors
        HandleColors();

        // Choosing which values to use
        double usedDistance;
        double neededAngleForTurretRotation = 0;
        if (shouldShootOnCamera) {
            usedDistance = distanceMeasuredCamera;
            neededAngleForTurretRotation += cameraAdder - rotationDegreesMeasuredCamera * cameraErrorMultiplier - robot.getMotorComponent("TurretRotateMotor").getPosition() * encoderMultiplier;
        }
        else {
            usedDistance = distanceToWallOdometry;
            neededAngleForTurretRotation -= rotationToWallOdometry; /// TODO this might be to be reversed in some ways
        }
        if(neededAngleForTurretRotation < -30) neededAngleForTurretRotation += 360;

        // pose resets
        if (gamepad1.dpad_up && gamepad1.dpad_right) robot.getFollowerInstance().instance().setPose(farStart);
        if (gamepad1.dpad_down && gamepad1.dpad_left) robot.getFollowerInstance().instance().setPose(new Pose(0,0,0));

        // Driver Intake
        wantsToIntakeDriver = robot.getControllerKey("A1").IsToggledOnPress;

        // Driver preparing for shooting
        isTryingToFire = robot.getControllerKey("Y1").IsToggledOnPress;

        // Driver actual firing with sorting
        wantsToFireSortingWithIntake = robot.getControllerKey("B1").IsToggledOnPress;

        // Driver Actual Shooting
        wantsToFireWithIntake = robot.getControllerKey("X1").IsToggledOnPress;
        needsToLowerGates = robot.getControllerKey("X1").ExecuteOnPress;

        // Driver Outputting
        wantsToOutput = robot.getControllerKey("RIGHT_BUMPER1").IsToggledOnPress;

        // ====================== Sorting Stuff ======================

        if(robot.getControllerKey("RIGHT_BUMPER2").ExecuteOnPress){
            ballColorQueue.add(BallColorSet_Decode.Purple);
            gamepad2.setLedColor(254,0,254,1000000);
        }
        if(robot.getControllerKey("LEFT_BUMPER2").ExecuteOnPress){
            ballColorQueue.add(BallColorSet_Decode.Green);
            gamepad2.setLedColor(0,254,0,1000000);
        }
        if(robot.getControllerKey("B2").ExecuteOnPress){
            ballColorQueue.clearQueue();
            gamepad2.setLedColor(254,254,254,1000000);
        }

        // Enable / Disable camera

        flashSensors(gamepad1.right_trigger > 0.4);

        // Slowdown
        if (robot.getControllerKey("LEFT_BUMPER1").ExecuteOnPress) {
            if (robot.getControllerKey("LEFT_BUMPER1").IsToggledOnPress) {
                robot.setDriveTrainSlowdown(0.3);
            } else robot.setDriveTrainSlowdown(1);
        }

        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Decision Making Code ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        if(wantsToOutput || wantsToFireWithIntake || wantsToFireSortingWithIntake) // only actually affect balls when you can output balls so you can count them even if they are stying on the hole and are undetected for a time
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
            if (!hasBallInLeftChamber) gateState = -1; // first fill up left
            else if (!hasBallInRightChamber) gateState = 1; // then right
            else gateState = -  1; // then continue pointing to right for when you fire
        }

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
        if(!wantsToFireSortingWithIntake){
            if (wantsToFireWithIntake) { // if normal is active
                if (needsToLowerGates) {
                    if(usedDistance > 2.9){
                        needsToLowerGates = false; // to not infi repeat
                        robot.executeNow(new ActionSequence(
                                new StateAction("RightGateServo", "OPEN"),
                                new DelayAction(200),
                                new StateAction("RightGateServo", "CLOSED"),

                                new DelayAction(timer1),

                                new StateAction("RightGateServo", "OPEN"),

                                new DelayAction(timer2),

                                new StateAction("LeftGateServo", "OPEN")
                        ));
                    }
                    else{
                        needsToLowerGates = false; // to not infi repeat
                        robot.executeNow(new ActionSequence(
                        new StateAction("LeftGateServo", "OPEN"),
                        new DelayAction(timer3),
                        new StateAction("RightGateServo", "OPEN")
                        ));
                    }

                }
            }
            else if (!needsToLowerGates) { // if neither is active
                needsToLowerGates = true; // to not infi repeat, will not trigger the one above due to wantToFireWithIntake
                if(wantsToOutput){
                    robot.executeNow(new StateAction("RightGateServo", "OPEN"));
                    robot.executeNow(new StateAction("LeftGateServo", "OPEN"));
                }
                else{
                    robot.executeNow(new StateAction("RightGateServo", "CLOSED"));
                    robot.executeNow(new StateAction("LeftGateServo", "CLOSED"));
                }
            }
        }
        else{
            if(wantsToFireWithIntake){// wtf why are they both active
                gamepad1.rumble(1,1,200);
            }
            else{ // if sorting firing is active

                if(sortingShootTimer.milliseconds() > 800){
                    shouldPullFromQueue = true;
                    sortingShootTimer.reset();
                }

                if(shouldPullFromQueue){
                    ballToFire = ballColorQueue.pull();

                    if(ballToFire == calculatedRightSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall){
                        robot.executeNow(new ActionSequence(
                                new StateAction("RightGateServo", "OPEN"),
                                new DelayAction(500),
                                new StateAction("RightGateServo", "CLOSED")
                        ));
                    }
                    else if(ballToFire == calculatedLeftSensorDetectedBall && ballToFire != BallColorSet_Decode.NoBall){
                        robot.executeNow(new ActionSequence(
                                new StateAction("LeftGateServo", "OPEN"),
                                new DelayAction(500),
                                new StateAction("LeftGateServo", "CLOSED")
                        ));
                    }
                    ballToFire = BallColorSet_Decode.NoBall;

                }


            }
        }

        // Outtake Stuff
        if (isTryingToFire) {
            // ----------------------- Power Stuff -----------------------

            double targetVelocity = distanceToVelocityFunction(usedDistance);
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Velocity)
                    .setTarget((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity));


            // ----------------------- Angle Stuff -----------------------

            double turretAngleVal = distanceToAngleFunction(usedDistance);
            turretAngleVal = clamp(turretAngleVal,262,324);
            robot.getServoComponent("TurretAngle")
                    .setTarget((eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal));


            // ----------------------- Rotation Stuff -----------------------
            if(usedDistance > 2.9) neededAngleForTurretRotation += farZoneCameraAdder;
            robot.getMotorComponent("TurretRotateMotor")
                    .setTarget(neededAngleForTurretRotation);
            
        }
        else {
            double targetVelocity = distanceToVelocityFunction(usedDistance);
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.Power)
                    .setTarget(0);

            double turretAngleVal = distanceToAngleFunction(usedDistance);
            robot.executeNow(new StateAction("TurretAngle", "DEFAULT")); // go to default position

            robot.getMotorComponent("TurretRotateMotor")
                    .setTarget(0);


        }

        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Telemetry and Overrides ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        RobotController.telemetry.addData("robot rotation", robot.getCurrentPose().getHeading());
        RobotController.telemetry.addData("robot Y", robot.getCurrentPose().getY());
        RobotController.telemetry.addData("robot X", robot.getCurrentPose().getX());
        RobotController.telemetry.addData("is outtake Overriden", robot.getMotorComponent("TurretSpinMotor").isOverriden());
        RobotController.telemetry.addData("actual Velocity", robot.getMotorComponent("TurretSpinMotor").getVelocity());
        RobotController.telemetry.addData("SPEED", robot.getMotorComponent("TurretSpinMotor").getPower());
    }


    // ============================ Init Stuff ============================


    @Override
    public void runOpMode() {
        // init
        setStuffToDefault();
        teamSensitiveStuff();

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

        while (opModeInInit()) {
            robot.init_loop();
        }
        robot.getFollowerInstance().setStartingPose(globalRobotPose);

        while (opModeIsActive()) {
            // loop
            robot.loop();
        }
        passPose();
    }

    public void teamSensitiveStuff() {
        teamPipeline = 0;
        currentTeamColor = TeamColor.Blue;
    }

    public void setStuffToDefault() { // occasionally copy paste declarations here so we don't have surprises
        ballCounter = 0;
        shouldShootOnCamera = false;
        shouldRemoveBalls = false;
        shouldPullFromQueue = false;
        if(ballColorQueue == null) ballColorQueue = new BallColorQueue();
        ballColorQueue.clearQueue();
        ballToFire = BallColorSet_Decode.NoBall;
        sortingShootTimer.reset();

        /// ----------------- Odometry Stuff -----------------
        distanceToWallOdometry = 0;
        rotationToWallOdometry = 0;

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

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class,"Control Hub");

        gamepad1.setLedColor(254,254,254,1000000);
        gamepad2.setLedColor(254,0,0,1000000);
    }


    // ============================ Color Stuff ============================

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


    // ============================ Camera Stuff ============================

    public void processCameraStuff() {
        LLResult llResult = limelight3A.getLatestResult();

        // rotation
        rotationDegreesMeasuredCamera = llResult.getTx();

        // distance

        double targetArea = llResult.getTa();

        RobotController.telemetry.addData("areaPercentage", targetArea);
        double a = 8.60403612;
        double b = -0.0119936722;

        distanceMeasuredCamera = Math.log(targetArea / a) / b;
    }

    // ============================ Odometry Stuff ============================

    public static double calculateHeadingAdjustment(Pose robotPose, double targetX, double targetY) {
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
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Normalize to [0, 360)
        if (robotHeadingDeg<0) robotHeadingDeg += 360;

        // Calculate smallest signed angle difference: [-180, 180]
        double angleDiff = targetAngleDeg - robotHeadingDeg;
        angleDiff = ((angleDiff + 540) % 360) - 180;  // neat trick for wrapping to [-180, 180]

        // Positive = target to the robot's right (clockwise turn), negative = to the left

        return angleDiff;
    }

    public static double calculateDistanceToWallInMeters(Pose robotPose, double targetX, double targetY) {
        return calculateDistance(robotPose, new Pose(targetX, targetY, 0), true);
    }


    // ============================ Weird Stuff ============================
    public Pose passPose() {
        globalRobotPose = robot.getFollowerInstance().instance().getPose();
        return globalRobotPose;
    }
    protected ElapsedTime flashingTimer = new ElapsedTime();
    public void flashSensors(boolean isFlashing){
        if(flashingTimer.milliseconds() % 200 > 100){
            robot.getColorSensorComponent("colorSensorRight").useSensorLight(false);
            robot.getColorSensorComponent("colorSensorLeft").useSensorLight(false);
        }
        else{
            robot.getColorSensorComponent("colorSensorRight").useSensorLight(true);
            robot.getColorSensorComponent("colorSensorLeft").useSensorLight(true);
        }
        if(!isFlashing){
            robot.getColorSensorComponent("colorSensorRight").useSensorLight(true);
            robot.getColorSensorComponent("colorSensorLeft").useSensorLight(true);
        }
    }
}
