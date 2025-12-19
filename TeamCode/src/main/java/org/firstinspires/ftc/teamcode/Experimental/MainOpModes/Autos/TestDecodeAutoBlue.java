package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.degreesToOuttakeTurretServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.greenSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.launchSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.normalizeTurretRotationForServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall1;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall2;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Disabled
@Autonomous(name = "Decode Down Auto Blue", group = "Tests")
public class TestDecodeAutoBlue extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private boolean startAuto = false;
    private boolean hasShooted = false;
    private boolean isShooting = false;
    private boolean turretHasBall = false;
    private boolean canSequence2, canSequence3, canSequence4;
    double ballCounter = 0;
    public static double targetX = 125;
    public static double targetY = 46;
    public static double publicAngleConstantThingTemp = -48;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime isFiringTimer = new ElapsedTime();
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

    public static double targetPower = 0.6;
    /// --------------------------------------------------------
    private Pose starter = new Pose( 0.0, 0.0, 0.0); // Default Start Position (p0)
    private Pose small_triangle_shoot = new Pose(5.64, -11.5, 0); // Pose1: shooting position small triangle
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
                ;
            }

            private void controls() { // this will happen in a loop
                boolean isMoving = robot.getFollowerInstance().getInstance().isBusy();
                countBalls();
                LoopChecks();
                if(startAuto) AutoSequence1();
                if (ballCounter == 1 && !isMoving() && isShooting) unstuckBallSequence();
                if(!unsticking && ballCounter >= 1 && !isShooting && !isMoving() && getErrorFromShooting() < 1 && !hasShooted) AutoShootingSequenceCloseTriangle();
                // to be continued if we ever get this far
                if(hasShooted && canSequence2 && !startAuto) AutoSequence2();
                if(hasShooted && canSequence3 && !canSequence2) AutoSequence3();
                if(hasShooted && canSequence4 && !canSequence3) AutoSequence4();
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
        canSequence2 = true; canSequence3 = true; canSequence4 = true;
        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple1 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor1");
        colorSensorPurple2 = hardwareMap.get(NormalizedColorSensor.class, "purplesensor2");
        colorSensorLaunch = hardwareMap.get(NormalizedColorSensor.class, "launchsensor");
    }

    private void unstuckBallSequence() {
        if (unsticking) return;
        unsticking = true;
        robot.addToQueue(new MoveAction(false, big_triangle_offset));
        robot.addToQueue(new MoveAction(false, big_triangle_shoot));
        unsticking = false;
    }

    private boolean isMoving() {
        return robot.getFollowerInstance().getInstance().getVelocity().getMagnitude() > 1;
    }

    private double getErrorFromShooting() {
        Pose current = robot.getFollowerInstance().getInstance().getPose();
        double x_err = Math.abs(big_triangle_shoot.getX() - current.getX());
        double y_err = Math.abs(big_triangle_shoot.getY() - current.getY());
        return x_err * y_err;
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
        double turretAngleVal = 61;
        robot.getServoComponent("TurretAngle").setOverrideTarget_bool(true);
        robot.addTelemetryData("turret angle estimation", turretAngleVal);
        robot.getServoComponent("TurretAngle").setOverrideTargetPos(degreesToOuttakeTurretServo(turretAngleVal));

        //velocity
        double targetVelocity = targetPower;
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverrideBoolean(false)
                .setOverrideCondition(true)
                .setPowerOverride(targetVelocity)
        ;

        if (turretHasBall && robot.getMotorComponent("TurretSpinMotor").getVelocity() > 1000 && isFiringTimer.milliseconds() > 800) {
            isFiringTimer.reset();
            robot.addToQueue(
                    new StateAction(false, "PurpleGateServo", "CLOSED"),
                    new DelayAction(true, 400),
                    new StateAction(true, "TransferServo", "UP"),
                    new DelayAction(true, 350),
                    new StateAction(true, "TransferServo", "DOWN"),
                    new StateAction(true, "PurpleGateServo", "OPEN")
            );
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
        launchSensorBall = BallColorSet_Decode.getColorForStorage(launchSensorColors);

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
        robot.addToQueue(new MoveAction(false, big_triangle_shoot)); // first shoot 3
        startAuto = false;
        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL"))
                .addToQueue(new StateAction(true, "IntakeSorterServo", "REDIRECT_TO_PURPLE")); // collecting
    }
    private void AutoSequence3() {
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverrideBoolean(false)
                .setOverrideCondition(true)
                .setPowerOverride(0)
        ;

        //robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL")); // collecting
        robot.addToQueue(new MoveAction(false, second_row_ready)); // go to collect
        robot.addToQueue(new MoveAction(false, second_row_done));
        robot.addToQueue(new MoveAction(false, second_row_ready));
        robot.addToQueue(new MoveAction(false, big_triangle_shoot));
        canSequence2 = false;
        hasShooted = false;
    }
    private void AutoSequence2() {
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverrideBoolean(false)
                .setOverrideCondition(true)
                .setPowerOverride(0)
        ;
        //robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL")); // collecting
        robot.addToQueue(new MoveAction(false, first_row_ready)); // go to collect
        robot.addToQueue(new MoveAction(false, first_row_done));
        robot.addToQueue(new MoveAction(false, first_row_ready));
        robot.addToQueue(new MoveAction(false, big_triangle_shoot));
        canSequence3 = false;
        hasShooted = false;
    }

    private void AutoSequence4() {
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverrideBoolean(false)
                .setOverrideCondition(true)
                .setPowerOverride(0)
        ;
        //robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL")); // collecting
        robot.addToQueue(new MoveAction(false, third_row_ready)); // go to collect
        robot.addToQueue(new MoveAction(false, third_row_done));
        robot.addToQueue(new MoveAction(false, third_row_ready));
        robot.addToQueue(new MoveAction(false, big_triangle_shoot));
        canSequence4 = false;
        hasShooted = false;
    }
}
