package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Visual.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;

@Config
@TeleOp(name="Main TeleOP Blue", group="Main")
public class MainTeleOP extends LinearOpMode {
    protected RobotController robot;
    protected TrajectoryCalculator trajectoryCalculator = new TrajectoryCalculator();
    protected VoltageSensor controlHubVoltageSensor;
    protected Limelight3A limelight3A = null;
    protected DcMotorEx externalEncoder = null;
    protected boolean sorterChoice = false;
    protected boolean G_P_P = false;
    protected boolean P_G_P = false;
    protected boolean P_P_G = false;
    protected boolean isInSortingPeriod = false;
    public static double powerMultiplier = 1;
    public static boolean absoluteZeroTurretRotation = false;
    public static double targetVelocity = 0;
    public static double targetTurret = 0;
    public static double lastTargetTurret = 0;
    public static double degreeSubtract = 2;
    public static double degreeSubtractAdder = -5;
    public static double degreeSubtractMulti = 1;
    public static boolean isTryingToFire = false;
    protected ElapsedTime isReadyToFireTimer = new ElapsedTime();
    protected ElapsedTime isFiringTimer = new ElapsedTime();
    public boolean isReadyToFire = false;
    public static  int purpleCounter = 0;
    public static int greenCounter = 0;
    public static boolean canFire = false;

    public static int ballCounter = 0;
    protected long timerForIntake = 0;
    protected long lastTimeNano = 0;
    protected long tick_ns = 0;
    public static double targetX = 125;
    public static double targetY = 46;
    public static Pose farStart = new Pose(120,24,Math.toRadians(90)); // no more reversing X
    public static double cameraErrorMultiplier = 1;
    public static double encoderMultiplier = 1;
    public static double cameraAdder = 0;
    public static double farZoneCameraAdder = 2;
    public static double targetVelFar = 1300;
    public static double targetPowerFar = 0.67;

    public static double targetAngleFar = 60;
    public static double targetAngleMid = 61;
    protected double id = 0;
    double[] pythonOutputs = {1, 2};
    double pos_y = pythonOutputs[1];
    public static double camera_error;
    public static double late_camera_error;
    public boolean turnOnCamera = true;
    public double last_power = 0;
    public double lastDistance = 0;
    public static double turretVelocityOverride = 0;
    public static double turretAngleOverride = 0;

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

    protected void robotMainLoop() {
        // all of the code

        double distanceToApriltag = getDistanceToAprilTag();
        robot.addTelemetryData("distance_to_april", distanceToApriltag);

        tick_ns = calculateTimeDiff();
        robot.addTelemetryData("loop time Milis",tick_ns / 1000000);
        HandleColors();
        camera_error = calculateCameraError();
        canFire = Math.abs(camera_error) <= 1 && camera_error != 0;
        late_camera_error = getLateCameraError();


        // intakeing

        robot.addTelemetryData("CAMERA ERROR", camera_error);

        if (timerForIntake + 600 < System.currentTimeMillis()) {
            robot.addToQueue(new StateAction(false, "IntakeSorterServo", "REDIRECT_TO_PURPLE"));
        }


        if(gamepad2.bWasPressed()) {
            turnOnCamera= !turnOnCamera;
        }
        if(gamepad1.dpad_up && gamepad1.dpad_right) robot.getFollowerInstance().getInstance().setPose(farStart);
        if(gamepad1.dpad_down && gamepad1.dpad_left) robot.getFollowerInstance().getInstance().setPose(new Pose(0,0,0));


        if (robot.getControllerKey("A1").IsToggledOnPress) {
            robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL"));
            if (
                    (purpleSensorBall1 == BallColorSet_Decode.Purple
                            || greenSensorBall == BallColorSet_Decode.Purple
                            || purpleSensorBall1 == BallColorSet_Decode.Green
                            || greenSensorBall == BallColorSet_Decode.Green)
                            && timerForIntake + 1000 < System.currentTimeMillis()
            ) {
                ballCounter++;
                timerForIntake = System.currentTimeMillis();
            }

            //decide where gate should be opened
            if (!isInSortingPeriod) {
                robot.addToQueue(new StateAction(false, "GreenGateServo", "CLOSED"));
                robot.addToQueue(new StateAction(false, "PurpleGateServo", "OPEN"));
            } else {
                if (purpleSensorBall1 == BallColorSet_Decode.Purple || greenSensorBall == BallColorSet_Decode.Purple) {
                    purpleCounter++;
                    if (purpleCounter > 2) {
                        robot.addToQueue(new StateAction(false,"IntakeMotor" ,"SLOW_REVERSE"));
                        purpleCounter--;
                        robot.addToQueue(new DelayAction(true,400));
                        robot.addToQueue(new StateAction(false,"IntakeMotor" ,"OFF"));
                    }
                    sorterChoice = true;
                }else {
                    greenCounter++;
                    if (greenCounter > 1) {
                        robot.addToQueue(new StateAction(false , "IntakeMotor","SLOW_REVERSE"));
                        greenCounter--;
                        robot.addToQueue(new DelayAction(true,400));
                        robot.addToQueue(new StateAction(false,"IntakeMotor" ,"OFF"));
                    }
                    sorterChoice = false;
                }
            }
        } else {
            robot.addToQueue(new StateAction(false, "IntakeMotor", "OFF"));
            robot.addToQueue(new StateAction(false, "IntakeSorterServo", "BLOCK"));
        }



        if (robot.getControllerKey("B1").IsToggledOnPress) {
            robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL_REVERSE"));
            robot.addToQueue(new StateAction(false, "IntakeSorterServo", "BLOCK"));
        }
        else if (robot.getMotorComponent("IntakeMotor").getPower() == -1)
            robot.addToQueue(new StateAction(false, "IntakeMotor", "OFF"));


        if (robot.getControllerKey("RIGHT_TRIGGER1").ExecuteOnPress && robot.getControllerKey("LEFT_TRIGGER1").ExecuteOnPress) {
            robot.setDirectionFlip(!robot.getDirectionFlip());
        }

        if (robot.getControllerKey("X1").ExecuteAfterPress) {
            // shoot
            robot.addToQueue(
                    new StateAction(false, "PurpleGateServo", "CLOSED"),
                    new DelayAction(true, 300),
                    new StateAction(true, "TransferServo", "UP"),
                    new DelayAction(true, 600),
                    new StateAction(true, "TransferServo", "DOWN"),
                    new StateAction(true, "PurpleGateServo", "OPEN")
            );
        }
        if (robot.getControllerKey("LEFT_BUMPER1").IsToggledOnPress) {
            robot.setDriveTrainSlowdown(0.3);
        } else robot.setDriveTrainSlowdown(1);

        if (robot.getControllerKey("RIGHT_BUMPER1").ExecuteOnPress) {
            if (robot.getControllerKey("RIGHT_BUMPER1").IsToggledOnPress) {
                robot
                        .addToQueue(new StateAction(true, "IntakeSorterServo", "REDIRECT_TO_PURPLE"))
                        .addToQueue(new StateAction(true, "IntakeMotor", "FULL_REVERSE"))
                ;
            } else {
                robot
                        .addToQueue(new StateAction(true, "IntakeSorterServo", "BLOCK"))
                        .addToQueue(new StateAction(true, "IntakeMotor", "OFF"))
                ;
            }
        }
        // ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == =DRIVER 2 ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  == =



        if (gamepad2.leftBumperWasPressed()) {
            powerMultiplier -= 0.05;
        }

        if (gamepad2.dpadDownWasPressed()) {
            resetEncoder();
        }

        if (gamepad2.rightBumperWasPressed()) {
            powerMultiplier += 0.05;
        }
        robot.addTelemetryData("POWER MULTIPLIER",powerMultiplier);
        if (robot.getControllerKey("Y1").ExecuteAfterPress) {
            isTryingToFire = !isTryingToFire;
            isReadyToFireTimer.reset();
        }
        isReadyToFire = (isReadyToFireTimer.milliseconds() > 1500 && isTryingToFire);

        if (gamepad2.aWasPressed()) {
            // take out a ball trough outtake
        }

        if (gamepad2.xWasPressed()) {
            absoluteZeroTurretRotation = !absoluteZeroTurretRotation;
        }

        if (gamepad2.dpadUpWasPressed()) {
                robot.addToQueue(new StateAction(false, "IntakeSorterServo", "PUSH_TO_PURPLE"));
                robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL_REVERSE"));
        }

        if (gamepad2.dpadLeftWasPressed()) {
            // override portita mov
            if (robot.getComponent("GreenGateServo").hasStateOfName("CLOSED"))
                robot.addToQueue(new StateAction(false, "GreenGateServo", "OPEN"));
            else robot.addToQueue(new StateAction(false, "GreenGateServo", "CLOSED"));
        }

        if (gamepad2.dpadRightWasPressed()) {
            // override portita verde
            if (robot.getComponent("PurpleGateServo").hasStateOfName("CLOSED"))
                robot.addToQueue(new StateAction(false, "PurpleGateServo", "OPEN"));
            else robot.addToQueue(new StateAction(false, "PurpleGateServo", "CLOSED"));
        }


        ///  ==  ==  ==  ==  ==  ==  ==  ==  == Telemetry and Overrides ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

        robot
                .addTelemetryData("robot rotation", robot.getCurrentPose().getHeading())
                .addTelemetryData("robot Y", robot.getCurrentPose().getY())
                .addTelemetryData("robot X", robot.getCurrentPose().getX())
                .addTelemetryData("target velocity", targetVelocity)
                .addTelemetryData("actual Velocity", robot.getMotorComponent("TurretSpinMotor").getVelocity())
                .addTelemetryData("SPEED", robot.getMotorComponent("TurretSpinMotor").getPower())
        ;
        //.addTelemetryData("Pinpoint actual pos", pinpointTurret.getHeading(AngleUnit.DEGREES))




        double distance = trajectoryCalculator.calculateDistance(robot.getCurrentPose(), new Pose(targetX, targetY, 0), true);
        if (distance <= 1) degreeSubtract = degreeSubtractMulti * (distance != 0 ? 1 / (distance - 0.1) : 0);
        else degreeSubtract = 0;
        degreeSubtract += degreeSubtractAdder;


        boolean shouldRawPow = false;
        double turretAngleVal = 0;
        if(isTryingToFire){
            //power and angle stuff

                distance = distanceToApriltag;
                if (Double.isInfinite(distance)) distance =lastDistance;
                lastDistance = distance;

                distance /= 100;

                if (distance > 2.55) {
                    turretAngleVal = targetAngleFar;
                    cameraAdder = farZoneCameraAdder;
                    shouldRawPow = true;
                }
                else if (distance > 0.8) {
                    turretAngleVal = targetAngleMid;
                    cameraAdder = 0;
                    shouldRawPow = false;
                }
                else {
                    turretAngleVal = -4.12746 * distance + 71.29151;
                    cameraAdder = 0;
                    shouldRawPow = false;
                }
                distance *= 100;

                //targetVelocity = 0.0001807 * Math.pow(distance, 3) - 0.077115 * distance * distance + 11.6851 * distance + 371.81972;
                targetVelocity = 1.13037 * distance + 835.31442;
                if (targetVelocity > 2500) {
                targetVelocity = last_power;
                }
                last_power = targetVelocity; // so that we avoid infinity

            // ==================== power setting ====================

            //targetVelocity *= voltageMultiplier(controlHubVoltageSensor.getVoltage());

            if(!shouldRawPow){
                robot.getMotorComponent("TurretSpinMotor")
                        .targetVPIDOverrideBoolean(true)
                        .setOverrideCondition(false)
                        .setTargetOverride((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                ;
            }
            else {
                targetVelocity = targetPowerFar;
                robot.getMotorComponent("TurretSpinMotor")
                        .targetVPIDOverrideBoolean(false)
                        .setOverrideCondition(true)
                        .setPowerOverride((eval(turretVelocityOverride) ? turretVelocityOverride : targetVelocity))
                ;
            }

            robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL"));

            // ==================== angle setting ====================

            turretAngleVal = clamp(turretAngleVal, 58.5, 72);
            robot.getServoComponent("TurretAngle")
                    .setOverrideTarget_bool(true)
                    .setOverrideTargetPos(degreesToOuttakeTurretServo((eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal)));

            // ==================== rotation stuff ====================

            targetTurret = calculateHeadingAdjustment(robot.getCurrentPose(), targetX, targetY);
            //if on camera
            if(eval(camera_error) && turnOnCamera) {
                if (Math.abs(camera_error) < 50) // if camera is seeing the tag and is not infinite
                    targetTurret = -getEncoderReadingFormatted() * encoderMultiplier -camera_error * cameraErrorMultiplier + cameraAdder;
                else {
                    targetTurret = lastTargetTurret;
                }
                lastTargetTurret = targetTurret;
            }

            robot.addTelemetryData("target on camera",getEncoderReadingFormatted() * encoderMultiplier + camera_error * cameraErrorMultiplier + cameraAdder);
            robot.addTelemetryData("target on odometry",calculateHeadingAdjustment(robot.getCurrentPose(), targetX, targetY));
            robot.addTelemetryData("targetY", targetY);

            if (absoluteZeroTurretRotation) targetTurret = 0;
            robot.getServoComponent("TurretRotateServo")
                    .setOverrideTarget_bool(true)
                    .setOverrideTargetPos(normalizeTurretRotationForServo(targetTurret));
        }
        else {
            //if not trying to fire

            targetVelocity = 0;
            robot.getMotorComponent("TurretSpinMotor")
                    .targetVPIDOverrideBoolean(false)
                    .setOverrideCondition(true)
                    .setPowerOverride(0)
            ;
            robot.getServoComponent("TurretRotateServo")
                    .setOverrideTarget_bool(true)
                    .setOverrideTargetPos(normalizeTurretRotationForServo(0));

            turretAngleVal = 63;
            robot.getServoComponent("TurretAngle")
                    .setOverrideTarget_bool(true)
                    .setOverrideTargetPos(degreesToOuttakeTurretServo((eval(turretAngleOverride) ? turretAngleOverride : turretAngleVal)));
        }


        robot.addTelemetryData("turret angle estimation", turretAngleVal);
        robot.addTelemetryData("Current Encoder Position", getEncoderReadingFormatted());
        robot.addTelemetryData("late camera error", late_camera_error);
        robot.addTelemetryData("voltage", controlHubVoltageSensor.getVoltage());
        robot.addTelemetryData("voltage multiplier ", voltageMultiplier(controlHubVoltageSensor.getVoltage()));
        robot.addTelemetryData("motor power", robot.getMotorComponent("TurretSpinMotor").getPower());
        robot.addTelemetryData("power on distance",getPowerOnDistance(getDistanceToAprilTag()));
        robot.addTelemetryData("actual velocity", robot.getMotorComponent("TurretSpinMotor").getVelocity());

    }

    public static boolean isActive = false;

    @Override
    public void runOpMode() {
        // init
        isActive = true;
        ballCounter = 0;
        powerMultiplier = 1;
        teamPipeline = 0;
        currentTeamColor = TeamColor.Blue;

        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {

                robotMainLoop();
            }
        };

        robot.init(OpModes.TeleOP);
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        InitOtherStuff(0);
        robot.UseDefaultMovement();
        camera_error = calculateCameraError();
        late_camera_error = getLateCameraError();

        while (opModeInInit()) {
            robot.init_loop();
        }
        robot.getFollowerInstance().setStartingPose(globalRobotPose);

        while (opModeIsActive()) {
            // loop
            robot.loop();
        }
        // stop
        isActive = false;
        passPose();
    }

    protected static double last_time = 0;

    protected double getLateCameraError() {
        double now = System.currentTimeMillis();
        if (now - last_time > 200) {
            late_camera_error = calculateCameraError();
            last_time = now;
        }
        return late_camera_error;
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

        externalEncoder = hardwareMap.get(DcMotorEx.class,"backpurple");

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class,"Control Hub");

        //controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }
    public double PurpleDetect = 0;
    public double GreenDetect = 0;

    protected void resetEncoder() {
        if (!isTryingToFire) {
            externalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            externalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


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
    protected double getDistanceToAprilTag() {
        LLResult llResult = limelight3A.getLatestResult();
        double targetArea = llResult.getTa();

        robot.addTelemetryData("areaPrecentage", targetArea);
        double a = 8.60403612;
        double b = -0.0119936722;

        return Math.log(targetArea / a) / b;
    }
    private double getPowerOnDistance(double dist) {
        return  1.13037*dist + 835.31442;
    }
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

    public double calculateCameraError() {
        LLResult llResult = limelight3A.getLatestResult();
        return llResult.getTx();
    }
    public long calculateTimeDiff() {
        long current = System.nanoTime();
        long laster = lastTimeNano;
        lastTimeNano = current;
        return current - laster;
    }
    public double getEncoderReadingFormatted() {
        if (externalEncoder == null) return 0;
        double reading = externalEncoder.getCurrentPosition() * -1 / 123.37;
//        while (reading < -180) reading += 360;
//        while (reading > 180) reading -= 360;
        return reading;
    }
    public Pose passPose() {
        globalRobotPose = robot.getFollowerInstance().getInstance().getPose();
        return globalRobotPose;
    }
}
