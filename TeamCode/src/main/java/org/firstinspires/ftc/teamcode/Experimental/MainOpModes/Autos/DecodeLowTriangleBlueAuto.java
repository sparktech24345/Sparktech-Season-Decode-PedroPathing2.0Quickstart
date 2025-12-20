package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.degreesToOuttakeTurretServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.greenSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.launchSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.normalizeTurretRotationForServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall1;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall2;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.calculateHeadingAdjustment;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.cameraErrorMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.encoderMultiplier;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.farZoneCameraAdder;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.targetAngleFar;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.targetAngleMid;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.targetVelFar;

import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos.DecodeLowTriangleBlueAuto.AutoEnum.*;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;
import java.util.List;

@Config
@Autonomous(name = "Auto Small Triangle BLUE", group = "Tests")
public class DecodeLowTriangleBlueAuto extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    protected Limelight3A limelight3A = null;
    protected DcMotorEx externalEncoder = null;
    private boolean startAuto = false;
    private boolean hasShooted = false;
    private boolean isShooting = false;
    private boolean turretHasBall = false;
    private boolean firstRowBool = false;
    private boolean secondRowBool = false;
    private boolean thirdRowBool = false;
    private boolean canShootFirstRow = false;
    private boolean canShootSecondRow = false;
    private boolean canShootThirdRow = false;
    private boolean timeToFire, canSequence3, parkBool, ballIsStuck, isMoving;
    double ballCounter = 0;
    public static double targetX = 125;
    public static double targetY = 46;
    public static double ballsLaunched = 0;
    public static double publicAngleConstantThingTemp = 21;
    public static double last_power = 0;
    public static double lastTargetTurret = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime isFiringTimer = new ElapsedTime();
    ElapsedTime shouldJiggle = new ElapsedTime();
    ElapsedTime shouldFinish = new ElapsedTime();
    private boolean had_balls = false;

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
    private boolean unsticking = false;
    public static double targetPower = 0.7;
    /// --------------------------------------------------------
    private Pose starter = new Pose(0.0, 0.0, 0.0); // Default Start Position (p0)
    private Pose small_triangle_shoot = new Pose(10, 0, 0); // Pose1: shooting position small triangle
    private Pose unstuckPose = new Pose(20, 6, 0); // Pose1: shooting position small triangle
    private Pose HP_collect = new Pose(38.6, -5.56, 0); // Pose3: HP collect
    private Pose first_row_ready = new Pose(52, 12, 90); // Pose4: collect first row right
    private Pose first_row_done = new Pose(52, 40, 90); // Pose5: collect first row left
    private Pose lever = new Pose(38.31, -62.65, 0); // Pose6: lever pose
    private Pose second_row_ready = new Pose(16, 76, 90); // Pose7: collect second row right
    private Pose second_row_done = new Pose(34, 77, 90); // Pose8: colect second row left
    private Pose big_triangle_shoot = new Pose(1, -90, 0); // Pose9: shooting big triangle pose
    private Pose big_triangle_offset = new Pose(1, -70, 0); // Pose9: shooting big triangle pose
    private Pose third_row_ready = new Pose(16, 100, 90); // Pose10: collect third row right
    private Pose third_row_done = new Pose(34, 99, 90); // Pose11: collect third row left
    private Pose classifier_starter = new Pose(28.355210672213335, 119.64113250492127, 0); // Pose12: start position from sorter

    @Override
    public void init() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                controls();
                telemetry();
            }

            private void telemetry() {
                robot
                        .addTelemetryData("robot rotation", Math.toDegrees(robot.getCurrentPose().getHeading()))
                        .addTelemetryData("robot Y", robot.getCurrentPose().getY())
                        .addTelemetryData("robot X", robot.getCurrentPose().getX())
                        .addTelemetryData("current velocity", robot.getMotorComponent("TurretSpinMotor").getVelocity())
                        .addTelemetryData("hasShooted", hasShooted)
                        .addTelemetryData("ballCounter", ballCounter)
                        .addTelemetryData("shootTimer_ms", shootTimer.milliseconds())
                ;
                robot.
                        addTelemetryData("isMoving", isMoving)
                        .addTelemetryData("timeToFire", timeToFire);
            }

            private void controls() { // this will happen in a loop
                isMoving = robot.getFollowerInstance().getInstance().isBusy();
                countBalls();
                checkSwitch(isMoving);


                //TODO:: -->>> UNCOMMENT THIS LINE OF CODE
                if ((timer.milliseconds() > 26000 || parkBool) && startAuto){
                    AutoPark();
                    terminateTurret();
                }
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
        timeToFire = true;
        canSequence3 = true;
        parkBool = false;
        ballsLaunched = 0;
        ballIsStuck = false;
        autoEnum = StartSequence;
        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple1 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor1");
        colorSensorPurple2 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor2");
        colorSensorLaunch = hardwareMap.get(NormalizedColorSensor.class, "launchsensor");

        externalEncoder = hardwareMap.get(DcMotorEx.class,"backpurple");
        ConvertPoses();
        FixTeamStuff();
    }

    private boolean isMoving() {
        return robot.getFollowerInstance().getInstance().getVelocity().getMagnitude() > 1;
    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
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

    private void countBalls() {

        greenSensorColors = colorSensorGreen.getNormalizedColors();
        purpleSensorColors1 = colorSensorPurple1.getNormalizedColors();
        purpleSensorColors2 = colorSensorPurple2.getNormalizedColors();
        launchSensorColors = colorSensorLaunch.getNormalizedColors();

        Color.colorToHSV(greenSensorColors.toColor(), hsvValuesGreen);
        Color.colorToHSV(purpleSensorColors1.toColor(), hsvValuesPurple1);
        Color.colorToHSV(purpleSensorColors2.toColor(), hsvValuesPurple2);
        Color.colorToHSV(launchSensorColors.toColor(), hsvValuesLaunch);

        greenSensorBall = BallColorSet_Decode.getColorForStorage(greenSensorColors);
        purpleSensorBall1 = BallColorSet_Decode.getColorForStorage(purpleSensorColors1.red * 1.5, purpleSensorColors1.green * 1.5, purpleSensorColors1.blue * 1.5);
        purpleSensorBall2 = BallColorSet_Decode.getColorForStorage(purpleSensorColors2);
        launchSensorBall = BallColorSet_Decode.getColorForTurret(launchSensorColors);

        robot.addTelemetryData("green sensor detect", greenSensorBall);
        robot.addTelemetryData("purple sensor1 detect", purpleSensorBall1);
        robot.addTelemetryData("purple sensor2 detect", purpleSensorBall2);
        robot.addTelemetryData("launch sensor detect", launchSensorBall);

        turretHasBall = (launchSensorBall != BallColorSet_Decode.NoBall);

        had_balls = eval(ballCounter);
        ballCounter = eval(greenSensorBall != BallColorSet_Decode.NoBall)
                + eval(purpleSensorBall1 != BallColorSet_Decode.NoBall)
                + eval(purpleSensorBall2 != BallColorSet_Decode.NoBall)
                + eval(launchSensorBall != BallColorSet_Decode.NoBall);
    }

    /// ============================== Sequence Stuff ===========================

    public enum AutoEnum{
        StartSequence,
        IsAfterStartSequence,
        UnstuckSequence,
        ParkingSequence,

    }
    public static AutoEnum autoEnum = StartSequence;
    public void checkSwitch(boolean isBusy){
        if(!isBusy && startAuto){
            switch (autoEnum) {
                case StartSequence:
                    AutoSequence1();
                    autoEnum = IsAfterStartSequence;
                    break;

                case UnstuckSequence:

                    break;

                case ParkingSequence: // parking stuff
                    parkBool = true;
                    break;
            }
        }
    }



    private void AutoSequence1() {
        robot
                .addToQueue(new MoveAction(false, small_triangle_shoot)) // first shoot 3
                .addToQueue(new StateAction(false, "IntakeMotor", "FULL"))
                .addToQueue(new StateAction(true, "IntakeSorterServo", "REDIRECT_TO_PURPLE")); // collecting
    }

    private void AutoSequenceUnstuck() {
        robot.addToQueue(new MoveAction(false, unstuckPose));
        robot.addToQueue(new MoveAction(true, small_triangle_shoot));
        canSequence3 = false;
        isFiringTimer.reset();
        ballIsStuck = false;
    }

    private void AutoSequenceFire() {
        robot.addToQueue(
                new MoveAction(false, small_triangle_shoot),
                new StateAction(true, "IntakeMotor")
        );
    }

    private void AutoSequenceFirstRow() {
        robot.addToQueue(
                new MoveAction(false, first_row_ready),
                new StateAction(false, "IntakeMotor", "FULL"),
                new StateAction(true, "IntakeSorterServo", "REDIRECT_TO_PURPLE"),
                new MoveAction(true, first_row_done),
                new DelayAction(true, 500),
                new StateAction(true, "IntakeSorterServo", "BLOCK"),
                new StateAction(true, "IntakeMotor", "OFF")
        );
        firstRowBool = false;
    }

    private void AutoSequenceSecondRow() {
        robot.addToQueue(
                new MoveAction(false, second_row_ready),
                new StateAction(true, "IntakeSorterServo", "REDIRECT_TO_PURPLE"),
                new StateAction(true, "IntakeMotor", "FULL"),
                new MoveAction(true, second_row_done),
                new DelayAction(true, 500),
                new StateAction(true, "IntakeSorterServo", "BLOCK"),
                new StateAction(true, "IntakeMotor", "OFF")
        );
        secondRowBool = false;
    }

    private void AutoSequenceThirdRow() {
        robot.addToQueue(
                new MoveAction(false, third_row_ready),
                new StateAction(true, "IntakeSorterServo", "REDIRECT_TO_PURPLE"),
                new StateAction(true, "IntakeMotor", "FULL"),
                new MoveAction(true, third_row_done),
                new DelayAction(true, 500),
                new StateAction(true, "IntakeSorterServo", "BLOCK"),
                new StateAction(true, "IntakeMotor", "OFF")
        );
        thirdRowBool = false;
    }

    private void AutoPark() {
        robot.addToQueue(new MoveAction(false, unstuckPose));
        parkBool = false;
        robot.addToQueue(new StateAction(false, "IntakeMotor", "OFF"));
        robot.getServoComponent("TurretRotateServo")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(normalizeTurretRotationForServo(0));

        robot.getMotorComponent("TurretSpinMotor")
                .targetVPIDOverrideBoolean(false)
                .setOverrideCondition(true)
                .setPowerOverride(0);


    }



    /// ============================ Firing Stuff ============================


    public double getDistanceToAprilTag(){
        //limelight3A.pipelineSwitch(teamPipeline);
        LLResult llResult = limelight3A.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        double targetArea = 0;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            List <List<Double>> bigList = fiducial.getTargetCorners();
            double cornern1X = bigList.get(0).get(0);
            double cornern1Y = bigList.get(0).get(1);
            double cornern2X = bigList.get(1).get(0);
            double cornern2Y = bigList.get(1).get(1);
            double cornern3X = bigList.get(2).get(0);
            double cornern3Y = bigList.get(2).get(1);
            double cornern4X = bigList.get(3).get(0);
            double cornern4Y = bigList.get(3).get(1);
            double calcualtedHeight = (Math.abs(cornern1Y - cornern4Y) + Math.abs(cornern2Y - cornern3Y)) / 2; // the average
            targetArea = (calcualtedHeight * calcualtedHeight)/(720*960)*100;
            }
        double a = 8.60403612;
        double b = -0.0119936722;

        return Math.log(targetArea / a) / b;
    }

    public double getEncoderReadingFormatted() {
        if (externalEncoder == null) return 0;
        double reading = externalEncoder.getCurrentPosition() * -1 / 123.37;
//        while (reading < -180) reading += 360;
//        while (reading > 180) reading -= 360;
        return reading;
    }

    public double calculateCameraError() {
        limelight3A.pipelineSwitch(teamPipeline);
        LLResult llResult = limelight3A.getLatestResult();
        return llResult.getTx();
    }
    private double getPowerOnDistance(double dist) {
        return 0.000551 * dist + 0.4816;
    }

    void calculateFiringStuff() {
        double targetVelocity = 0;
        double distanceOnCamera = getDistanceToAprilTag();
        double power = 0;
        double turretAngleVal = 63;
        double cameraAdder = 0;


        if (distanceOnCamera > 255) {
            power = targetVelFar; // only fire from the tip of the triangle
            turretAngleVal = targetAngleFar;
            cameraAdder = farZoneCameraAdder;
        }
        else if(distanceOnCamera > 90) {
            power =  getPowerOnDistance(distanceOnCamera);
            turretAngleVal = targetAngleMid;
            cameraAdder = 0;
        }
        else{
            power =  getPowerOnDistance(distanceOnCamera);
            turretAngleVal = -4.12746 * (distanceOnCamera / 100) + 71.29151;
            cameraAdder = 0;
        }

        if(power > 1){
            power = last_power;
        }
        last_power = power; // so that we avoid infinity
        targetVelocity = power;



        // ==================== power setting ====================

        //targetVelocity *= voltageMultiplier(controlHubVoltageSensor.getVoltage());

        robot.getMotorComponent("TurretSpinMotor")
                //            .targetOverride(true)
                .targetVPIDOverrideBoolean(true)
                .setOverrideCondition(false)
                .setTargetOverride(targetVelocity)
        ;
        robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL"));

        // ==================== angle setting ====================

        turretAngleVal = clamp(turretAngleVal, 58.5, 72);
        robot.getServoComponent("TurretAngle")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(degreesToOuttakeTurretServo(turretAngleVal));

        // ==================== rotation stuff ====================

        double targetTurret = calculateHeadingAdjustment(robot.getCurrentPose(), targetX, targetY);
        double camera_error = calculateCameraError();
        //if on camera
        if(eval(camera_error)){
            if (Math.abs(camera_error) < 50) // if camera is seeing the tag and is not infinite
                targetTurret = -getEncoderReadingFormatted() * encoderMultiplier -camera_error * cameraErrorMultiplier + cameraAdder;
            else{
                targetTurret = lastTargetTurret;
            }
            lastTargetTurret = targetTurret;
        }

        robot.addTelemetryData("target on camera",getEncoderReadingFormatted() * encoderMultiplier + camera_error * cameraErrorMultiplier + cameraAdder);
        robot.addTelemetryData("target on odometry",calculateHeadingAdjustment(robot.getCurrentPose(), targetX, targetY));

        robot.getServoComponent("TurretRotateServo")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(normalizeTurretRotationForServo(targetTurret));



    }

    void terminateTurret() {
        robot.getMotorComponent("TurretSpinMotor")
                .targetVPIDOverrideBoolean(false)
                .setOverrideCondition(true)
                .setPowerOverride(0)
        ;
        robot.getServoComponent("TurretRotateServo")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(normalizeTurretRotationForServo(0));

        double turretAngleVal = 63;
        robot.getServoComponent("TurretAngle")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(degreesToOuttakeTurretServo(turretAngleVal));
    }



    /// ============================ Pose Stuff ============================



    public Pose passPose() {
        globalRobotPose = robot.getFollowerInstance().getInstance().getPose();
        return globalRobotPose;
    }

    public Pose ModifyPose(Pose pose) {
        return pose;
    }

    public void ConvertPoses() {
        starter = ModifyPose(starter);
        small_triangle_shoot = ModifyPose(small_triangle_shoot); // Pose1: shooting position small triangle
        unstuckPose = ModifyPose(unstuckPose); // Pose1: shooting position small triangle
        HP_collect = ModifyPose(HP_collect); // Pose3: HP collect
        first_row_ready = ModifyPose(first_row_ready); // Pose4: collect first row right
        first_row_done = ModifyPose(first_row_done); // Pose5: collect first row left
        lever = ModifyPose(lever); // Pose6: lever pose
        second_row_ready = ModifyPose(second_row_ready); // Pose7: collect second row right
        second_row_done = ModifyPose(second_row_done); // Pose8: colect second row left
        big_triangle_shoot = ModifyPose(big_triangle_shoot); // Pose9: shooting big triangle pose
        big_triangle_offset = ModifyPose(big_triangle_offset); // Pose9: shooting big triangle pose
        third_row_ready = ModifyPose(third_row_ready); // Pose10: collect third row right
        third_row_done = ModifyPose(third_row_done); // Pose11: collect third row left
        classifier_starter = ModifyPose(classifier_starter); // Pose12: start position from sorter
    }

    public void FixTeamStuff() {
        // init
        currentTeamColor = TeamColor.Blue;
        teamPipeline = 0;

        //farZoneCameraAdder = farZoneCameraAdder;
        //targetY = targetY;
    }
}
