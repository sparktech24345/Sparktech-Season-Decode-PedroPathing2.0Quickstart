package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.degreesToOuttakeTurretServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.greenSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.launchSensorBall;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.normalizeTurretRotationForServo;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall1;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.purpleSensorBall2;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.calculateHeadingAdjustment;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name = "Decode Down Auto Blue", group = "Tests")
public class TestDecodeAutoBlue extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private boolean startAuto = false;
    private boolean hasShooted = false;
    private boolean isShooting = false;
    private boolean turretHasBall = false;
    private boolean canSequence2, canSequence3, canSequence4;
    double ballCounter =0;
    public static double targetX = 125;
    public static double targetY = 46;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime isFiringTimer = new ElapsedTime();

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
    /// --------------------------------------------------------


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
                        ;
            }

            private void controls() { // this will happen in a loop
                boolean isMoving = robot.getFollowerInstance().getInstance().isBusy();
                countBalls();
                LoopChecks();
                if(startAuto) AutoSequence1();
                if(ballCounter >= 3 && !isShooting && !isMoving) AutoShootingSequenceCloseTriangle();
//                // to be continued if we ever get this far
//                if(hasShooted && canSequence2) AutoSequence2();
//                if(hasShooted && canSequence3) AutoSequence3();
//                if(hasShooted && canSequence4) AutoSequence4();
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
        MakeAutoStates();
        robot.getFollowerInstance().setStartingPose(new Pose(0, 0, 90));
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
        canSequence2 = true; canSequence3 = true; canSequence4 = true;
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
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    private void MakeAutoStates() {
        robot.addAutoPosition("pose0", 0.0, 0.0, 0.0); // Default Start Position (p0)
        robot.addAutoPosition("shooting_small_triangle", -8.03860461618018, -7.573567788431964, 90); // Pose1: shooting position small triangle
        robot.addAutoPosition("hp_collect", -5.6340994046429005, 41.52871439776083, 90); // Pose3: HP collect
        robot.addAutoPosition("collect_row1_right", -51.43651767039862, 11.520354503721704, 90); // Pose4: collect first row right
        robot.addAutoPosition("collect_row1_left", -53.903058870570874, 44.08911517285926, 90); // Pose5: collect first row left
        robot.addAutoPosition("lever_pose", -62.716122124138785, 38.33424004982776, 90); // Pose6: lever pose
        robot.addAutoPosition("collect_row2_right", -75.56306613711861, 13.0993808536079, 90); // Pose7: collect second row right
        robot.addAutoPosition("collect_row2_left", -76.425911009781, 36.33493738850271, 90); // Pose8: colect second row left
        robot.addAutoPosition("shooting_big_triangle", -74.05108594518947, -9.54259917492003, 90); // Pose9: shooting big triangle pose
        robot.addAutoPosition("collect_row3_right", -99.2199034202756, 14.359903410663756, 90); // Pose10: collect third row right
        robot.addAutoPosition("collect_row3_left", -99.35278704785925, 35.598865418922244, 90); // Pose11: collect third row left
        robot.addAutoPosition("start_from_sorter", -121.51062492310533, 29.559189203217276, 90); // Pose12: start position from sorter

// Pose2 and Pose13 were excluded based on your instructions ("invalid" and "ignore, error from start measured").
    }
    private void LoopChecks(){
        if(ballCounter == 0 && shootTimer.milliseconds() > 400) hasShooted = true;
        if(ballCounter > 0) hasShooted = false;

    }
    private void AutoShootingSequenceCloseTriangle(){
        ///shooty shooty

        // rotation
        double targetTurret = calculateHeadingAdjustment(robot.getCurrentPose(), targetX, targetY);
        robot.getServoComponent("TurretRotateServo")
                .setOverrideTarget_bool(true)
                .setOverrideTargetPos(normalizeTurretRotationForServo(targetTurret));


        //angle
        double turretAngleVal = 61;
        robot.getServoComponent("TurretAngle").setOverrideTarget_bool(true);
        robot.addTelemetryData("turret angle estimation", turretAngleVal);
        robot.getServoComponent("TurretAngle").setOverrideTargetPos(degreesToOuttakeTurretServo(turretAngleVal));

        //velocity
        double targetVelocity = 1400;
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverride(true)
                .setTargetOverride(targetVelocity);

        if (turretHasBall &&  robot.getMotorComponent("TurretSpinMotor").getVelocity() > 1350 && isFiringTimer.milliseconds() > 800) {
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
    private void countBalls(){

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

        turretHasBall = (launchSensorBall != BallColorSet_Decode.NoBall);

        ballCounter = eval(greenSensorBall != BallColorSet_Decode.NoBall)
                + eval(purpleSensorBall1 != BallColorSet_Decode.NoBall)
                + eval(purpleSensorBall2 != BallColorSet_Decode.NoBall)
                + eval(launchSensorBall != BallColorSet_Decode.NoBall);
    }
    private void AutoSequence1(){
        robot.addToQueue(new MoveAction(false, "shooting_big_triangle")); // first shoot 3
        startAuto = false;
    }
    private void AutoSequence2(){
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverride(true)
                .setTargetOverride(0);

        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL")); // collecting
        robot.addToQueue(new MoveAction(true, "collect_row2_right")); // go to collect
        robot.addToQueue(new MoveAction(true, "collect_row2_left"));
        robot.addToQueue(new DelayAction(true,1000));
        robot.addToQueue(new MoveAction(true, "shooting_big_triangle"));
        canSequence2 = false;
    }
    private void AutoSequence3(){
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverride(true)
                .setTargetOverride(0);
        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL")); // collecting
        robot.addToQueue(new MoveAction(true, "collect_row1_right")); // go to collect
        robot.addToQueue(new MoveAction(true, "collect_row1_left"));
        robot.addToQueue(new DelayAction(true,1000));
        robot.addToQueue(new MoveAction(true, "shooting_big_triangle"));
        canSequence3 = false;
    }

    private void AutoSequence4(){
        robot.getMotorComponent("TurretSpinMotor")
                .targetOverride(true)
                .setTargetOverride(0);
        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL")); // collecting
        robot.addToQueue(new MoveAction(true, "collect_row3_right")); // go to collect
        robot.addToQueue(new MoveAction(true, "collect_row3_left"));
        robot.addToQueue(new DelayAction(true,1000));
        robot.addToQueue(new MoveAction(true, "shooting_big_triangle"));
        canSequence4 = false;
    }
}
