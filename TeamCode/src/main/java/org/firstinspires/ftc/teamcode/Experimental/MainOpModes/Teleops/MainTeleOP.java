package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Visual.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Config
@TeleOp(name="Main TeleOP", group="Main")
public class MainTeleOP extends LinearOpMode {

    private RobotController robot;
    private TrajectoryCalculator trajectoryCalculator = new TrajectoryCalculator();
    private GoBildaPinpointDriver pinpointTurret;
    private  DcMotorEx turretEncoder;
    private VoltageSensor controlHubVoltageSensor;
    private Limelight3A limelight3A;
    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private boolean sorterChoice = false;
    private boolean isInSortingPeriod = false;
    public static double adderTurretRotateForTests = 0;
    public static double servoP = 0.005 ; //0.0135
    public static double servoI = 0;
    public static double servoD = 0.00085; //0.001
    public static double servoVel = 5000;
    public static double servoAcel = 10000;
    public static double servoTime = 1;
    public static double cameraError = 0;
    public static double targetVelocity = 0;
    public static double rpmMultiplier = 0;
    public static double rpmDefault = 1500;
    public static double rpmExponent = 0;
    public static double rpmkp = 2;
    public static double rpmkd = 1;
    public static double targetTurret = 0;
    public static double degreeSubtract = 2;
    public static double degreeSubtractAdder = -5;
    public static double degreeSubtractMulti = 1;
    public static boolean isTryingToFire = false;
    public boolean isReadyToFire = false;
    public static  int purpleCounter = 0;
    public static int greenCounter = 0;

    public static int ballCounter = 0;
    private long timerForIntake = 0;
    private long lastTimeNano = 0;
    private double nonCorrectedCameraError = 0;
    private long tick_ns = 0;
    public static Pose targetPose = new Pose(125,42.8,0);

    /// ----------------- Color Sensor Stuff ------------------
    private NormalizedColorSensor colorSensorGreen;
    private NormalizedColorSensor colorSensorPurple;
    private NormalizedRGBA greenSensorColors;
    private NormalizedRGBA purpleSensorColors;


    final float[] hsvValuesGreen = new float[3];
    final float[] hsvValuesPurple = new float[3];
    /// --------------------------------------------------------

    @Override
    public void runOpMode() {
        // init
        ballCounter = 0;

        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                // all of the code
                long startTime = System.currentTimeMillis();
                tick_ns = calculateTimeDiff();
                robot.addTelemetryData("loop time Nano", tick_ns);
                robot.addTelemetryData("loop time Milis",tick_ns / 1000000);
                //HandleColors();
                getPinpointTurretPosition();

                robot.addTelemetryData("Checkpint After pinpoint and coloers", System.currentTimeMillis() - startTime);
                // intakeing

                robot.addTelemetryData("tester",robot.getComponent("IntakeMotor").getPosition());

                if (false && (purpleSensorBall == BallColorSet_Decode.Purple || greenSensorBall == BallColorSet_Decode.Purple || purpleSensorBall == BallColorSet_Decode.Green || greenSensorBall == BallColorSet_Decode.Green)) {
                    if (ballCounter > 3 && robot.getComponent("IntakeMotor").getPosition() != -1) { //dont infinite stack comands if full reversing already
                        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL_REVERSE"));
                        robot.addToQueue(new DelayAction(true, 400));
                        robot.addToQueue(new StateAction(true, "IntakeMotor", "OFF"));
                        robot.addTelemetryData("reversing",true);
                        ballCounter--;
                    }
                }
                if (ballCounter >= 3  && timerForIntake + 600 < System.currentTimeMillis() && false) {
                    //robot.addToQueue(new StateAction(false, "IntakeSorterServo", "BLOCK"));
                    //robot.addTelemetryData("blocking",true);
                }else if (timerForIntake + 600 < System.currentTimeMillis()) {
                    robot.addToQueue(new StateAction(false, "IntakeSorterServo", "REDIRECT_TO_PURPLE"));
                    robot.addTelemetryData("redirecting",true);
                }

                if (robot.getControllerKey("A1").IsToggledOnPress) {
                    robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL"));
                    if (
                            (purpleSensorBall == BallColorSet_Decode.Purple
                            || greenSensorBall == BallColorSet_Decode.Purple
                            || purpleSensorBall == BallColorSet_Decode.Green
                            || greenSensorBall == BallColorSet_Decode.Green)
                            && timerForIntake + 1000 < System.currentTimeMillis()
                    ) {
                        ballCounter++;
                        timerForIntake = System.currentTimeMillis();
                    }

                    //decide where gate should be opened
                    if (!isInSortingPeriod) {
                        //robot.addToQueue(new StateAction(false, "IntakeSorterServo", "REDIRECT_TO_PURPLE"));
                        robot.addToQueue(new StateAction(false, "GreenGateServo", "CLOSED"));
                        robot.addToQueue(new StateAction(false, "PurpleGateServo", "OPEN"));
                    } else {
                        if (purpleSensorBall == BallColorSet_Decode.Purple || greenSensorBall == BallColorSet_Decode.Purple) {
                            purpleCounter++;
                            if (purpleCounter>2) {
                                robot.addToQueue(new StateAction(false,"IntakeMotor" ,"SLOW_REVERSE"));
                                purpleCounter--;
                                robot.addToQueue(new DelayAction(true,400));
                                robot.addToQueue(new StateAction(false,"IntakeMotor" ,"OFF"));
                            }
                            sorterChoice = true;
                        }else {
                            greenCounter++;
                            if (greenCounter>1) {
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

                robot.addTelemetryData("ball counter",ballCounter);


                if (robot.getControllerKey("B1").IsToggledOnPress) {
                    robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL_REVERSE"));
                    robot.addToQueue(new StateAction(false, "IntakeSorterServo", "BLOCK"));
                }
                else if(robot.getMotorComponent("IntakeMotor").getPower() == -1)
                    robot.addToQueue(new StateAction(false, "IntakeMotor", "OFF"));

                if (robot.getControllerKey("RIGHT_BUMPER1").ExecuteOnPress) {
                    setDriveTrainSlowdown(0.2);
                } else setDriveTrainSlowdown(1);

                if (robot.getControllerKey("RIGHT_TRIGGER1").ExecuteOnPress && robot.getControllerKey("LEFT_TRIGGER1").ExecuteOnPress) {
                    setDirectionFlip(!getDirectionFlip());
                }

                if (robot.getControllerKey("X1").ExecuteAfterPress) {
                    // shoot
                    robot.addToQueue(
                            new StateAction(false, "PurpleGateServo", "CLOSED"),
                            new DelayAction(true, 600),
                            new StateAction(false, "TransferServo", "UP"),
                            new DelayAction(true, 600),
                            new StateAction(true, "TransferServo", "DOWN"),
                            new StateAction(false, "PurpleGateServo", "OPEN")
                    );
                }

                if (gamepad1.right_bumper) {
                    // output trough intake
                } if (gamepad1.leftBumperWasPressed()) {
//                    //false is green true is purple
//                    if (sorterChoice) {
//                        robot.addToQueue(new StateAction(false, "TurretSpinMotor", "FULL"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "CLOSED"));
//                        robot.addToQueue(new DelayAction(true, 2000));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL"));
//                        robot.addToQueue(new DelayAction(true, 1400));
//                        //robot.addToQueue(new StateAction(true, "PurpleGateServo", "CLOSED"));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "OFF"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "UP"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "DOWN"));
//                        robot.addToQueue(new StateAction(true, "TurretSpinMotor", "OFF"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "OPEN"));
//                    }
//                    else {
//                        robot.addToQueue(new StateAction(false, "TurretSpinMotor", "FULL"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "CLOSED"));
//                        robot.addToQueue(new DelayAction(true, 2000));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL"));
//                        robot.addToQueue(new DelayAction(true, 1400));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "CLOSED"));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "OFF"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "UP"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "DOWN"));
//                        robot.addToQueue(new StateAction(true, "TurretSpinMotor", "OFF"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "OPEN"));
//                    }
                }
//                if (gamepad1.rightBumperWasPressed()) {
//                    sorterChoice = !sorterChoice;
//                    if (sorterChoice) {
//                        //is now purple
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "CLOSED"));
//                    }
//                    else {
//                        //is now green
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "CLOSED"));
//                    }
//                }
                // ================================= DRIVER 2 ===============================================

                if (gamepad2.yWasPressed()) {
                    // Sorting toggle
                    isInSortingPeriod = !isInSortingPeriod;
                }

                if (gamepad2.leftBumperWasPressed()) {
                    // odometry false
                }

                if (gamepad2.rightBumperWasPressed()) {
                    // camera targeting false
                }

                if (gamepad2.xWasPressed() || robot.getControllerKey("Y1").ExecuteAfterPress) {
                    isTryingToFire = !isTryingToFire;
                }

                if (gamepad2.aWasPressed()) {
                    // take out a ball trough outtake
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

                robot.addTelemetryData("Checkpint After logic", System.currentTimeMillis() - startTime);

                ///  ================== Telemetry and Overrides ======================

                double cameraCorrectionError = calculateCameraError(targetTurret - robot.getCRServoComponent("TurretRotate").getServoAvrgPosition());
                robot
                        .addTelemetryData("turret power", robot.getCRServoComponent("TurretRotate").getPower())
                        .addTelemetryData("analog position", robot.getCRServoComponent("TurretRotate").getAnalogPosition())
                        .addTelemetryData("total analog position", robot.getCRServoComponent("TurretRotate").getServoAnalogTotalPosition())
                        .addTelemetryData("estimated calculated power", robot.getCRServoComponent("TurretRotate").getCalculatedPower())
                        .addTelemetryData("estimated error",targetTurret - robot.getCRServoComponent("TurretRotate").getServoAvrgPosition())
                        .addTelemetryData("Camera Error correction",cameraCorrectionError)
                        .addTelemetryData("robot rotation", robot.getCurrentPose().getHeading())
                        .addTelemetryData("robot Y", robot.getCurrentPose().getY())
                        .addTelemetryData("robot X", robot.getCurrentPose().getX())
                        .addTelemetryData("turret rotation estimation", calculateHeadingAdjustment(robot.getCurrentPose(), targetPose.getX(), targetPose.getY()))
                        .addTelemetryData("target velocity", targetVelocity)
                        .addTelemetryData("actual Velocity", robot.getMotorComponent("TurretSpinMotor").getVelocity())
                        .addTelemetryData("SPEEDD", robot.getMotorComponent("TurretSpinMotor").getPower())
                        .addTelemetryData("Pinpoint actual pos", pinpointTurret.getHeading(AngleUnit.DEGREES))
                        .addTelemetryData("Pinpoint calculated pos", robot.getCRServoComponent("TurretRotate").getpinpointTotalPosition());
                //robot.addTelemetryData("turret VERTICAL ANGLE",trajectoryCalculator.calcAngle(robot.getCurrentPose(),targetPose,true,motorRpm));

                robot.getMotorComponent("TurretSpinMotor").setRPMPIDconstants(rpmkp,0, rpmkd);

                targetTurret = calculateHeadingAdjustment(robot.getCurrentPose(),targetPose.getX(),targetPose.getY()) + gamepad1.right_stick_x * 30 + adderTurretRotateForTests;
                robot.getCRServoComponent("TurretRotate")
                        .setPIDconstants(servoP, servoI, servoD)
                        .setMotionconstants(servoVel, servoAcel, servoTime)
                        .setOverrideBool(true);


                double distance = trajectoryCalculator.calculateDistance(robot.getCurrentPose(), targetPose,true);
                if (distance <= 1) degreeSubtract = degreeSubtractMulti * (distance != 0 ? 1 / (distance - 0.1) : 0);
                else degreeSubtract = 0;
                degreeSubtract += degreeSubtractAdder;
                robot.addTelemetryData("degreesSubtract", degreeSubtract);


                robot.getServoComponent("TurretAngle").setOverrideTarget_bool(true);
                double turretAngleVal = trajectoryCalculator.findLowestSafeTrajectory(robot.getCurrentPose(), targetPose, true).getOptimalAngleDegrees() - degreeSubtract;
                robot.addTelemetryData("turret angle estimation", turretAngleVal);
                robot.getServoComponent("TurretAngle").setOverrideTargetPos(degreesToOuttakeTurretServo(turretAngleVal));
                //robot.getServoComponent("TurretAngle").setOverrideTargetPos(degreesToOuttakeTurretServo(trajectoryCalculator.calcAngle(robot.getCurrentPose(),targetPose,true,motorRpm)));


                if (isTryingToFire) {
                    //puterea calculat,unghiul calculat,rotatia calculata;
                    //motorRpm = degreesToOuttakeTurretServo(trajectoryCalculator.findLowestSafeTrajectory(robot.getCurrentPose(),targetPose,true).getMinInitialVelocity()) * rpmMultiplier;
                    robot.addTelemetryData("distance to wall", distance);
                    double dist = distance * 100; //in cm
                    targetVelocity = grade0 + dist * grade1 + dist * dist * grade2 + Math.pow(dist, 3) * grade3;
                    robot.getMotorComponent("TurretSpinMotor").targetOverride(true);
                    robot.getMotorComponent("TurretSpinMotor").setTargetOverride(targetVelocity);

                    robot.getCRServoComponent("TurretRotate").setTargetOverride(targetTurret);
                }
                else {
                    robot.getMotorComponent("TurretSpinMotor").targetOverride(false);
                    robot.addToQueue(new StateAction(false, "TurretSpinMotor","OFF"));
                    targetVelocity = 0;
                    robot.getCRServoComponent("TurretRotate").setTargetOverride(targetTurret); //TODO to set this to 0
                }

                robot.addTelemetryData("Checkpoint After telemetry", System.currentTimeMillis() - startTime);
            }
        };

        MakeComponents();
        MakeStates();
        InitOtherStuff();
        robot.UseDefaultMovement();

        while (opModeInInit()) {
            robot.init_loop();
        }

        while (opModeIsActive()) {
            // loop
            robot.loop();
        }
        // stop
    }
    public void InitOtherStuff(){
        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple = hardwareMap.get(NormalizedColorSensor.class, "purplesensor");
        pinpointTurret = hardwareMap.get(GoBildaPinpointDriver.class, "pinpointturret");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

        //controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        aprilTagWebcam.init(hardwareMap, robot.getTelemetryInstance(),"Webcam 1");

        pinpointTurret.resetPosAndIMU();
        pinpointTurret.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    }

    private void MakeComponents() {
        robot.makeComponent("IntakeMotor", new MotorComponent()
                .addMotor("intakemotor")
                .useWithPIDController(false)
                .useWithEncoder(false)
                .setRange(-1, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
        );

        robot.makeComponent("TurretSpinMotor", new MotorComponent()
                .addMotor("turretspin")
                .useWithPIDController(false)
                .setRPM_PIDCoefficients(0.005, 0.00055, 0)
                .setTargetOverride(0)
                .useWithEncoder(false)
                .setRange(-1, 1)
        );

        robot.makeComponent("TurretRotate", new CRServoComponent()
                .addMotor("turretrotateleft")
                .setEncoder("leftturretreader")
                .addMotor("turretrotateright")
                .useWithPIDController(true)
                .setPIDconstants(0, 0, 0)
                .setDirection("turretrotateleft", DcMotorSimple.Direction.REVERSE)
                .setDirection("turretrotateright", DcMotorSimple.Direction.REVERSE)
                .setRange(-270, 270) // range for PID
                .moveDuringInit(false)
        );

        robot.makeComponent("IntakeSorterServo", new ServoComponent()
                .addMotor("intakeservo")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("PurpleGateServo", new ServoComponent()
                .addMotor("purplegate")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("GreenGateServo", new ServoComponent()
                .addMotor("greengate")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TransferServo", new ServoComponent()
                .addMotor("transferservo")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TurretAngle", new ServoComponent()
                .addMotor("turretangle")
                .setOverrideTarget_bool(false) //go to init pos
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );
    }

    private void MakeStates() {
        robot.getComponent("IntakeMotor")
                .addState("OFF", 0, true)
                .addState("SLOW", 0.5)
                .addState("FULL", 1)
                .addState("FULL_REVERSE", -1)
                .addState("SLOW_REVERSE", -0.5);

        robot.getComponent("TurretSpinMotor")
                .addState("OFF", 0, true)
                .addState("FULL", 1);

        robot.getComponent("PurpleGateServo")
                .addState("OPEN", 15, true)
                .addState("CLOSED", 122);

        robot.getComponent("GreenGateServo")
                .addState("OPEN", 30)
                .addState("CLOSED", 200, true);

        robot.getComponent("IntakeSorterServo")
                .addState("REDIRECT_TO_PURPLE", 161.64)
                .addState("REDIRECT_TO_GREEN", 35)
                .addState("BLOCK", 93.816, true);

        robot.getComponent("TransferServo")
                .addState("DOWN", 30, true)
                .addState("UP", 230);

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 350, false) // 77 degrees looky
                .addState("UP_MAX", 192, true); // 50 degrees looky

        robot.addRobotState("TransferGreen", new RobotState(
                make_pair("GreenGateServo", "OPEN"),
                make_pair("TransferServo", "DOWN"),
                make_pair("IntakeMotor", "FULL")
        ));
        robot.addRobotState("TransferPurple", new RobotState(
                make_pair("PurpleGateServo", "OPEN"),
                make_pair("TransferServo", "DOWN"),
                make_pair("IntakeMotor", "FULL")
        ));
        robot.addRobotState("TransferUp", new RobotState(
                make_pair("IntakeMotor", "OFF"),
                make_pair("PurpleGateServo", "CLOSED"),
                make_pair("GreenGateServo", "CLOSED"),
                make_pair("TransferServo", "UP")
        ));
    } private void HandleColors() {


        greenSensorColors =colorSensorGreen.getNormalizedColors();
        purpleSensorColors =colorSensorPurple.getNormalizedColors();

        Color.colorToHSV(greenSensorColors.toColor(), hsvValuesGreen);
        Color.colorToHSV(purpleSensorColors.toColor(), hsvValuesPurple);

        greenSensorBall = BallColorSet_Decode.getColor(greenSensorColors);
        purpleSensorBall = BallColorSet_Decode.getColor(purpleSensorColors);


        robot.addTelemetryData("G_RED",(double)greenSensorColors.red);
        robot.addTelemetryData("G_BLUE",(double)greenSensorColors.blue);
        robot.addTelemetryData("G_GREEN",(double)greenSensorColors.green);

        robot.addTelemetryData("P_RED",(double)purpleSensorColors.red);
        robot.addTelemetryData("P_BLUE",(double)purpleSensorColors.blue);
        robot.addTelemetryData("P_GREEN",(double)purpleSensorColors.green);


        greenSensorBall = BallColorSet_Decode.NoBall;
        purpleSensorBall = BallColorSet_Decode.NoBall;

        robot.addTelemetryData("GREEN_SENSOR_BALL", greenSensorBall);
        robot.addTelemetryData("PURPLE_SENSOR_BALL", purpleSensorBall);
    }
    public double calculateHeadingAdjustment(Pose robotPose, double targetX, double targetY) {
        // Current robot position
        double x = robotPose.getX();
        double y = robotPose.getY(); // don't invert Y unless your coordinate system specifically requires it

        // Vector from robot to target
        double dx = targetX - x;
        double dy = targetY - y;

        // Angle from robot to target (in radians → degrees)
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Normalize to [0, 360)
        if (targetAngleDeg < 0) targetAngleDeg += 360;

        // Robot heading in degrees
        // Assuming robotPose.getHeading() is in radians, 0° = facing +X, increases counterclockwise
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Normalize to [0, 360)
        if (robotHeadingDeg < 0) robotHeadingDeg += 360;

        // Calculate smallest signed angle difference: [-180, 180]
        double angleDiff = targetAngleDeg - robotHeadingDeg;
        angleDiff = ((angleDiff + 540) % 360) - 180;  // neat trick for wrapping to [-180, 180]

        // Positive = target to the robot's right (clockwise turn), negative = to the left
        robot.addTelemetryData("CALCUL Robot heading", robotHeadingDeg);
        robot.addTelemetryData("CALCUL Target angle", targetAngleDeg);
        robot.addTelemetryData("CALCUL Heading adjustment", angleDiff);

        return angleDiff;
    }

    public static double degreesToOuttakeTurretServo(double degrees) {
        double m = 0.1709;
        double b = 17.19;
        double result = (degrees - b) / m;
        return clamp(result, 100, 359);
    }

    public void getPinpointTurretPosition() {
        pinpointTurret.update();
        if (pinpointTurret != null) {
            double diffAngleFromDriveTrain = pinpointTurret.getHeading(AngleUnit.DEGREES) - Math.toDegrees(robot.getCurrentPose().getHeading());
            robot.getCRServoComponent("TurretRotate").setPinpointPosition(diffAngleFromDriveTrain);
        }
        else robot.getCRServoComponent("TurretRotate").setPinpointPosition(0);
    }

    public double calculateCameraError(double existingError) {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        if (id20 != null) aprilTagWebcam.displayDetectionTelemetry(id20);

        if (id20 != null) cameraError = id20.ftcPose.x; //might be the wrong thing
        else cameraError = existingError;

        // telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
        double actualError = cameraError - existingError;

        nonCorrectedCameraError = cameraError;
        return clamp(actualError, -20, 20);
    }
    public long calculateTimeDiff() {
        long current = System.nanoTime();
        long laster = lastTimeNano;
        lastTimeNano = current;
        return current - laster;
    }
}
