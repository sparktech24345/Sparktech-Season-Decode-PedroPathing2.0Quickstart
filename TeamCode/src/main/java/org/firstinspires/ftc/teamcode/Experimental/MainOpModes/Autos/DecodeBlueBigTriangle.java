package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "Decode Auto blue Big Triangle", group = "Tests")
public class DecodeBlueBigTriangle extends OpMode {
    public static Pose farStart = new Pose(-120,27,Math.toRadians(90));
    public static Pose outside = new Pose(-105,27,Math.toRadians(90));
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
    public static double publicAngleConstantThingTemp = 20;
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
    private Pose starter = new Pose( 0.0, 0.0, 0.0); // Default Start Position (p0)
    private Pose small_triangle_shoot = new Pose(-10, 0, 0); // Pose1: shooting position small triangle
    private Pose unstuckPose = new Pose(-20, 6, 0); // Pose1: shooting position small triangle
    private Pose HP_collect = new Pose(-38.6, -5.56, 0); // Pose3: HP collect
    private Pose first_row_ready = new Pose(-15, 52, 0); // Pose4: collect first row right
    private Pose first_row_done = new Pose(-30, 52, 0); // Pose5: collect first row left
    private Pose lever = new Pose(-38.31, -62.65, 0); // Pose6: lever pose
    private Pose second_row_ready = new Pose(-15, 77, 0); // Pose7: collect second row right
    private Pose second_row_done = new Pose(-30, 77, 0); // Pose8: colect second row left
    private Pose big_triangle_shoot = new Pose(-1, -90, 0); // Pose9: shooting big triangle pose
    private Pose big_triangle_offset = new Pose(-1, -70, 0); // Pose9: shooting big triangle pose
    private Pose third_row_ready = new Pose(-15, 100, 0); // Pose10: collect third row right
    private Pose third_row_done = new Pose(-30, 100, 0); // Pose11: collect third row left
    private Pose classifier_starter = new Pose(-28.355210672213335, 119.64113250492127, 0); // Pose12: start position from sorter

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
                if(startAuto) AutoSequence();
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        robot.getFollowerInstance().getInstance().setPose(farStart);
        recorder = new AutoRecorder(true);
        timeToFire = true; canSequence3 = true; canSequence4 = true; ballsLaunched = 0; ballIsStuck = false;
        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple1 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor1");
        colorSensorPurple2 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor2");
        colorSensorLaunch = hardwareMap.get(NormalizedColorSensor.class, "launchsensor");
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
    private void AutoSequence(){
        robot.addToQueue(new MoveAction(false,outside));
        startAuto = false;
    }
    public Pose passPose(){
        globalRobotPose = robot.getFollowerInstance().getInstance().getPose();
        return globalRobotPose;
    }
}
