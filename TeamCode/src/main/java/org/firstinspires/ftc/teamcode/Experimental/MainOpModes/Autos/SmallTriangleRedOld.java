package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.degreesToOuttakeTurretServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.globalRobotPose;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.greenSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.launchSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.normalizeTurretRotationForServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall1;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall2;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;

@Config
@Autonomous(name = "OLD small triangle RED", group = "Tests")
public class SmallTriangleRedOld extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private boolean startAuto = false;
    private boolean hasShooted = false;
    private boolean isShooting = false;
    private boolean turretHasBall = false;
    private boolean timeToFire, canSequence3, canSequence4,ballIsStuck,isMoving;
    double ballCounter = 0;
    public static double targetX = 125;
    public static double targetY = 46;
    public static double ballsLaunched = 0;
    public static double publicAngleConstantThingTemp = -22;
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
    public static double targetPower = 0.75;
    /// --------------------------------------------------------
    private Pose starter = new Pose( 0.0, 0.0, 0.0); // Default Start Position (p0)
    private Pose small_triangle_shoot = new Pose(10, 0, 0); // Pose1: shooting position small triangle
    private Pose unstuckPose = new Pose(20, -6, 0); // Pose1: shooting position small triangle
    private Pose HP_collect = new Pose(38.6, 5.56, 0); // Pose3: HP collect
    private Pose first_row_ready = new Pose(15, -52, 0); // Pose4: collect first row right
    private Pose first_row_done = new Pose(30, -52, 0); // Pose5: collect first row left
    private Pose lever = new Pose(38.31, 62.65, 0); // Pose6: lever pose
    private Pose second_row_ready = new Pose(15, -77, 0); // Pose7: collect second row right
    private Pose second_row_done = new Pose(30, -77, 0); // Pose8: colect second row left
    private Pose big_triangle_shoot = new Pose(1, 90, 0); // Pose9: shooting big triangle pose
    private Pose big_triangle_offset = new Pose(1, 70, 0); // Pose9: shooting big triangle pose
    private Pose third_row_ready = new Pose(15, -100, 0); // Pose10: collect third row right
    private Pose third_row_done = new Pose(30, -100, 0); // Pose11: collect third row left
    private Pose classifier_starter = new Pose(28.355210672213335, -119.64113250492127, 0); // Pose12: start position from sorter

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
                        .addTelemetryData("current velocity",robot.getMotorComponent("TurretSpinMotor").getVelocity())
                        .addTelemetryData("hasShooted", hasShooted)
                        .addTelemetryData("ballCounter", ballCounter)
                        .addTelemetryData("shootTimer_ms", shootTimer.milliseconds())
                //if(!isMoving && timeToFire && !startAuto)
                ;
                robot.
                        addTelemetryData("isMoving", isMoving)
                        .addTelemetryData("timeToFire", timeToFire)
                        .addTelemetryData("start auto", startAuto);
            }

            private void controls() { // this will happen in a loop
                isMoving = robot.getFollowerInstance().getInstance().isBusy();
                if(isMoving) isFiringTimer.reset();
                countBalls();
                if(startAuto) AutoSequence1();
                if(!isMoving && timeToFire && !startAuto){
                    timeToFire = false;
                    isFiringTimer.reset();
                }
                if(!isMoving) AutoShootingSequenceCloseTriangle();
                if(ballsLaunched <= 2 && isFiringTimer.milliseconds() > 5000) ballIsStuck = true;
                if(ballIsStuck) AutoSequence3();
                if(timer.milliseconds()>22000 && canSequence4) AutoPark();
//                if (ballCounter == 1 && !isMoving() && isShooting) unstuckBallSequence();
//                if(!unsticking && ballCounter >= 1 && !isShooting && !isMoving() && getErrorFromShooting() < 1 && !hasShooted) AutoShootingSequenceCloseTriangle();
//                // to be continued if we ever get this far
//                if(hasShooted && canSequence2 && !startAuto) AutoSequence2();
//                if(hasShooted && canSequence3 && !canSequence2) AutoSequence3();
//                if(hasShooted && canSequence4 && !canSequence3) AutoSequence4();
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
        timeToFire = true; canSequence3 = true; canSequence4 = true; ballsLaunched = 0; ballIsStuck = false;
        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple1 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor1");
        colorSensorPurple2 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor2");
        colorSensorLaunch = hardwareMap.get(NormalizedColorSensor.class, "launchsensor");
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
        robot.getFollowerInstance().setStartingPose(starter);
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
    private void LoopChecks(){
        if (!eval(ballCounter)) {
            if (had_balls) {
                shootTimer.reset();
            } else if (shootTimer.milliseconds() > 2000) {
                hasShooted = true;
            }
        }

        if (ballCounter > 0) {
            hasShooted = false;
            shootTimer.reset();
        }

    }
    private void AutoShootingSequenceCloseTriangle(){
        ///shooty shooty

        // rotation
        double targetTurret = publicAngleConstantThingTemp;
        robot.getServoComponent("TurretRotateServo")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(normalizeTurretRotationForServo(targetTurret));

        //angle
        double turretAngleVal = 58;
        robot.getServoComponent("TurretAngle").setOverrideTarget_bool(true);
        robot.addTelemetryData("turret angle estimation", turretAngleVal);
        robot.getServoComponent("TurretAngle").setOverrideTargetPos(degreesToOuttakeTurretServo(turretAngleVal));

        //velocity
        double targetVelocity = targetPower;
        robot.getMotorComponent("TurretSpinMotor")
                .targetVPIDOverrideBoolean(false)
                .setOverrideCondition(true)
                .setPowerOverride(targetVelocity)
        ;

        if (turretHasBall && isFiringTimer.milliseconds() > 3000 && !isMoving) {
            isFiringTimer.reset();
            robot.addToQueue(
                    new StateAction(false, "PurpleGateServo", "CLOSED"),
                    new DelayAction(true, 400),
                    new StateAction(true, "TransferServo", "UP"),
                    new DelayAction(true, 350),
                    new StateAction(true, "TransferServo", "DOWN"),
                    new StateAction(true, "PurpleGateServo", "OPEN")
            );
            ballsLaunched++;
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
    private void AutoSequence1() {
        robot.addToQueue(new MoveAction(false, small_triangle_shoot)); // first shoot 3
        startAuto = false;
        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL"))
                .addToQueue(new StateAction(true, "IntakeSorterServo", "REDIRECT_TO_PURPLE")); // collecting
    }
    private void AutoSequence3(){
        robot.addToQueue(new MoveAction(false, unstuckPose));
        robot.addToQueue(new MoveAction(true, small_triangle_shoot));
        canSequence3 = false;
        isFiringTimer.reset();
        ballIsStuck = false;
    }
    private void AutoPark(){
        robot.addToQueue(new MoveAction(false, unstuckPose));
        canSequence4 = false;
        robot.addToQueue(new StateAction(false, "IntakeMotor", "OFF"));
        robot.getServoComponent("TurretRotateServo")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(normalizeTurretRotationForServo(0));

        robot.getMotorComponent("TurretSpinMotor")
                .setOverrideCondition(true)
                .setPowerOverride(0);


    }
    public Pose passPose(){
        globalRobotPose = robot.getFollowerInstance().getInstance().getPose();
        return globalRobotPose;
    }

}