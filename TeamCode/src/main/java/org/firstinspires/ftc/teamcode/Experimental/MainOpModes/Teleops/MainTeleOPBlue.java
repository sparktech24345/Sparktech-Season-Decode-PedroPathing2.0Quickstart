package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.calculateDistance;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.degreesToOuttakeTurretServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToAngleFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.distanceToVelocityFunction;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.greenSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.launchSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.normalizeTurretRotationForServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall1;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall2;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
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
    public static Pose farStart = new Pose(120,24,Math.toRadians(90)); // no more reversing X

    /// ----------------- Color Sensor Stuff ------------------
    protected NormalizedColorSensor colorSensorGreen;
    protected NormalizedColorSensor colorSensorPurple1;
    protected NormalizedColorSensor colorSensorPurple2;
    protected NormalizedColorSensor colorSensorLaunch;
    protected NormalizedRGBA greenSensorColors;
    protected NormalizedRGBA purpleSensorColors1;
    protected NormalizedRGBA purpleSensorColors2;
    protected NormalizedRGBA launchSensorColors;
    final float[] hsvValuesGreen = new float[3];
    final float[] hsvValuesPurple1 = new float[3];
    final float[] hsvValuesPurple2 = new float[3];
    final float[] hsvValuesLaunch = new float[3];
    public static int ballCounter = 0;
    public static boolean hasBallInRightChamber = false;
    public static boolean hasBallInLeftChamber = false;

    /// ----------------- Limelight Stuff -----------------
    protected Limelight3A limelight3A = null;
    public static double distanceMeasuredCamera =0;
    public static double rotationDegreesMeasuredCamera =0;
    public static double cameraErrorMultiplier = 1;
    public static double encoderMultiplier = 1;
    public static double cameraAdder = 0;
    public static double farZoneCameraAdder = 2;
    public static boolean shouldShootOnCamera = false;

    /// ----------------- Odometry Stuff -----------------
    public static double distanceToWallOdometry  =0;
    public static double rotationToWallOdometry  =0;

    /// ----------------- Intake Priorities -----------------

    public static boolean wantsToIntakeDriver = false;
    public static boolean wantsToIntakeLaunching = false;
    public static boolean wantsToFireWithIntake = false;
    public static boolean wantsToOutput = false;

    /// ----------------- Outtake Priorities -----------------
    public static double turretAngleOverride = 0;
    public static double turretVelocityOverride = 0;

    public static boolean isTryingToFire = false;
    public static boolean needsToLowerGates = false;
    protected void robotMainLoop() {
        // all of the code

        // processing
        processCameraStuff(teamPipeline);
        distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(),targetX,targetY);
        rotationToWallOdometry = calculateHeadingAdjustment(robot.getCurrentPose(),targetX,targetY);

        // Choosing which values to use
        double usedDistance=0;
        double neededAngleForTurretRotation=0;
        if(!shouldShootOnCamera){
            usedDistance = distanceToWallOdometry;
            neededAngleForTurretRotation = -getEncoderReadingFormatted() * encoderMultiplier + rotationToWallOdometry; /// TODO this might be to be reversed in some ways
        }
        else{
            usedDistance = distanceMeasuredCamera;
            neededAngleForTurretRotation = -getEncoderReadingFormatted() * encoderMultiplier -rotationDegreesMeasuredCamera * cameraErrorMultiplier + cameraAdder;
        }



        // pose resets
        if(gamepad1.dpad_up && gamepad1.dpad_right) robot.getFollowerInstance().getInstance().setPose(farStart);
        if(gamepad1.dpad_down && gamepad1.dpad_left) robot.getFollowerInstance().getInstance().setPose(new Pose(0,0,0));

        // Driver Intake
        wantsToIntakeDriver = robot.getControllerKey("A1").IsToggledOnPress;

        // Driver preparing for shooting
        isTryingToFire = robot.getControllerKey("B1").IsToggledOnPress;
        wantsToIntakeLaunching = isTryingToFire;

        // Driver Actual Shooting
        wantsToFireWithIntake = robot.getControllerKey("X1").IsToggledOnPress;
        needsToLowerGates =  robot.getControllerKey("X1").ExecuteOnPress;

        // Driver Outputting
        wantsToOutput = robot.getControllerKey("RIGHT_BUMPER1").IsToggledOnPress;

        // Enable / Disable camera


        // Slowdown
        if (robot.getControllerKey("LEFT_BUMPER1").IsToggledOnPress) {
            robot.setDriveTrainSlowdown(0.3);
        } else robot.setDriveTrainSlowdown(1);


        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Decision Making Code ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        // Intake stuff
        int intakeState =0; // if -1 then output if 0 then do nothing if 1 then intake
        int gateState =0; // if -1 then gate To The Left if 0 then do block if 1 then to the right
        if(!wantsToIntakeDriver && !wantsToIntakeLaunching && !wantsToFireWithIntake && !wantsToOutput){
            intakeState = 0;
            gateState = 0; // if doing nothing
        }
        if(wantsToOutput){
            wantsToIntakeDriver = wantsToIntakeLaunching = wantsToFireWithIntake = false; // make all of them false for override
            intakeState = -1;
            if(hasBallInRightChamber) gateState = 1; // open to the right so you can outtake
            else if(hasBallInLeftChamber) gateState = -1; // left is lower priority but checked
            else gateState = 0; // output last one fast
        }
        if(wantsToIntakeDriver || wantsToIntakeLaunching || wantsToFireWithIntake){ // for now keep them in the same bucket
            intakeState = 1;
            if (!hasBallInRightChamber) gateState = 1; // first fill up right
            else if(!hasBallInLeftChamber) gateState = -1; // then left
            else gateState = 1; // then continue pointing to right for when you fire
        }

        switch (intakeState){
            case -1:
                robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL_REVERSE"));
                break;

            case 0:
                robot.addToQueue(new StateAction(false, "IntakeMotor", "OFF"));
                break;

            case 1:
                robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL"));
                break;
        }
        switch (gateState){
            case -1:
                robot.addToQueue(new StateAction(false, "IntakeSorterServo", "REDIRECT_TO_LEFT"));
                break;

            case 0:
                robot.addToQueue(new StateAction(false, "IntakeSorterServo", "BLOCK"));
                break;

            case 1:
                robot.addToQueue(new StateAction(false, "IntakeSorterServo", "REDIRECT_TO_RIGHT"));
                break;
        }




        //firing logic
        if(wantsToFireWithIntake){
            if(needsToLowerGates){
                needsToLowerGates = false; // to not infi repeat
                robot.addToQueue(new StateAction(false, "RightGateServo", "OPEN"));
                robot.addToQueue(new DelayAction(true,200));
                robot.addToQueue(new StateAction(true, "LeftGateServo", "OPEN"));
            }
        }
        else if(needsToLowerGates){
            needsToLowerGates = true; // to not infi repeat, will not trigger the one abover due to wantToFireWithIntake
            robot.addToQueue(new StateAction(false, "RightGateServo", "CLOSED"));
            robot.addToQueue(new StateAction(false, "LeftGateServo", "CLOSED"));
        }

        // Outtake Stuff
        if(isTryingToFire){
            // ----------------------- Power Stuff -----------------------

            double targetVelocity = distanceToVelocityFunction(usedDistance);
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.VPIDOverride)
                    .setVPIDTarget((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity));


            // ----------------------- Angle Stuff -----------------------

            double turretAngleVal = distanceToAngleFunction(usedDistance);
            robot.getServoComponent("TurretAngle")
                    .setOperationMode(ServoComponent.ServoModes.positionOverride)
                    .setOverrideTargetPos(degreesToOuttakeTurretServo((eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal)));


            // ----------------------- Rotation Stuff -----------------------

            robot.getMotorComponent("TurretRotateServo")
                    .setOperationMode(MotorComponent.MotorModes.PIDToPositionOverride)
                    .setPositionPIDTarget(neededAngleForTurretRotation);


        }
        else{
            double targetVelocity = distanceToVelocityFunction(usedDistance);
            robot.getMotorComponent("TurretSpinMotor")
                    .setOperationMode(MotorComponent.MotorModes.VPIDOverride)
                    .setVPIDTarget(0);

            double turretAngleVal = distanceToAngleFunction(usedDistance);
            robot.getServoComponent("TurretAngle")
                    .setOperationMode(ServoComponent.ServoModes.executeState);
            robot.addToQueue(new StateAction(false, "TurretAngle", "DEFAULT")); // go to default position

            robot.getMotorComponent("TurretRotateServo")
                    .setOperationMode(MotorComponent.MotorModes.PIDToPositionOverride)
                    .setPositionPIDTarget(0);


        }

        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Telemetry and Overrides ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        robot
                .addTelemetryData("robot rotation", robot.getCurrentPose().getHeading())
                .addTelemetryData("robot Y", robot.getCurrentPose().getY())
                .addTelemetryData("robot X", robot.getCurrentPose().getX())
                .addTelemetryData("actual Velocity", robot.getMotorComponent("TurretSpinMotor").getVelocity())
                .addTelemetryData("SPEED", robot.getMotorComponent("TurretSpinMotor").getPower())
        ;

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

    public void teamSensitiveStuff(){
        teamPipeline = 0;
        currentTeamColor = TeamColor.Blue;
    }

    public void setStuffToDefault(){ // occasionally copy paste declarations here so we dont have surprises
        ballCounter = 0;
        shouldShootOnCamera = false;

        /// ----------------- Odometry Stuff -----------------
        distanceToWallOdometry  =0;
        rotationToWallOdometry  =0;

        /// ----------------- Intake Priorities -----------------

        wantsToIntakeDriver = false;
        wantsToIntakeLaunching = false;
        wantsToFireWithIntake = false;
        wantsToOutput = false;

        /// ----------------- Outtake Priorities -----------------
        turretAngleOverride = 0;
        turretVelocityOverride = 0;

        isTryingToFire = false;
        needsToLowerGates = false;
    }

    public void InitOtherStuff(int limelightPipeline) {
        //limelight stuff
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(limelightPipeline);
        limelight3A.reloadPipeline();
        limelight3A.setPollRateHz(100); // poll 100 times per second
        limelight3A.start();

        //other stuff like color sensor
        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple1 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor1");
        colorSensorPurple2 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor2");
        colorSensorLaunch = hardwareMap.get(NormalizedColorSensor.class, "launchsensor");

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class,"Control Hub");
    }


    // ============================ Color Stuff ============================

     protected void HandleColors() {


        greenSensorColors = colorSensorGreen.getNormalizedColors();
        purpleSensorColors1 = colorSensorPurple1.getNormalizedColors();
        purpleSensorColors2 = colorSensorPurple2.getNormalizedColors();
        launchSensorColors = colorSensorLaunch.getNormalizedColors();

        Color.colorToHSV(greenSensorColors.toColor(), hsvValuesGreen);
        Color.colorToHSV(purpleSensorColors1.toColor(), hsvValuesPurple1);
        Color.colorToHSV(purpleSensorColors2.toColor(), hsvValuesPurple2);
        Color.colorToHSV(launchSensorColors.toColor(), hsvValuesLaunch);


        greenSensorBall = BallColorSet_Decode.getColor(greenSensorColors);
        purpleSensorBall1 = BallColorSet_Decode.getColor(purpleSensorColors1);
        purpleSensorBall2 = BallColorSet_Decode.getColor(purpleSensorColors2);
        launchSensorBall = BallColorSet_Decode.getColorForTurret(launchSensorColors);

//        robot.addTelemetryData("G_RED",(double)greenSensorColors.red * 10000.0);
//        robot.addTelemetryData("G_BLUE",(double)greenSensorColors.blue * 10000.0);
//        robot.addTelemetryData("G_GREEN",(double)greenSensorColors.green * 10000.0);
//
//        robot.addTelemetryData("P1_RED",(double)purpleSensorColors1.red * 10000.0);
//        robot.addTelemetryData("P1_BLUE",(double)purpleSensorColors1.blue * 10000.0);
//        robot.addTelemetryData("P1_GREEN",(double)purpleSensorColors1.green * 10000.0);
//
//        robot.addTelemetryData("P2_RED",(double)purpleSensorColors2.red * 10000.0);
//        robot.addTelemetryData("P2_BLUE",(double)purpleSensorColors2.blue * 10000.0);
//        robot.addTelemetryData("P2_GREEN",(double)purpleSensorColors2.green * 10000.0);
//
//        robot.addTelemetryData("L_RED",(double)launchSensorColors.red * 10000.0);
//        robot.addTelemetryData("L_BLUE",(double)launchSensorColors.blue * 10000.0);
//        robot.addTelemetryData("L_GREEN",(double)launchSensorColors.green * 10000.0);
//
//        robot.addTelemetryData("G Sensed Color", BallColorSet_Decode.getColorForStorage(greenSensorColors));
//        robot.addTelemetryData("P1 Sensed Color", BallColorSet_Decode.getColorForStorage(purpleSensorColors1.red * 1.5, purpleSensorColors1.green * 1.5, purpleSensorColors1.blue * 1.5));
//        robot.addTelemetryData("P2 Sensed Color", BallColorSet_Decode.getColorForStorage(purpleSensorColors2));
//        robot.addTelemetryData("L Sensed Color", BallColorSet_Decode.getColorForStorage(launchSensorColors));


        telemetry.addData("G_SENSOR_BALL", greenSensorBall);
        telemetry.addData("P1_SENSOR_BALL", purpleSensorBall1);
        telemetry.addData("P2_SENSOR_BALL", purpleSensorBall1);
        telemetry.addData("L_SENSOR_BALL", launchSensorBall);
    }


    // ============================ Camera Stuff ============================

    public void processCameraStuff(int currentPipeline){
        LLResult llResult = limelight3A.getLatestResult();

        // rotation
        rotationDegreesMeasuredCamera = llResult.getTx();

        // distance

        double targetArea = llResult.getTa();

        robot.addTelemetryData("areaPrecentage", targetArea);
        double a = 8.60403612;
        double b = -0.0119936722;

        distanceMeasuredCamera =  Math.log(targetArea / a) / b;
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

    public static double calculateDistanceToWallInMeters(Pose robotPose, double targetX, double targetY){
        return calculateDistance(robotPose, new Pose(targetX, targetY, 0), true);
    }


    // ============================ Weird Stuff ============================
    public double getEncoderReadingFormatted() {
        double reading = robot.getMotorComponent("TurretRotateMotor").getPosition() * -1 / 123.37;
        return reading;
    }
    public Pose passPose() {
        globalRobotPose = robot.getFollowerInstance().getInstance().getPose();
        return globalRobotPose;
    }
}
