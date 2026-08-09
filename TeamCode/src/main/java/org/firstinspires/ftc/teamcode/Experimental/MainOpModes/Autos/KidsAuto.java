package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.MAX_DISTANCE_MM;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.MAX_VOLTS;
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
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.timeToTurnAirSortOff;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.usedTargetX;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.usedTargetY;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.ballInAirTime;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateDistanceToWallInMeters;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.calculateHeadingAdjustment;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.cameraAngle;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.mainTimerForSorting;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timer2;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timerBothOnOneChannelTimerForSorting;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.timerToCloseGate;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.vMultiplier;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.ActionSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.HoldAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BallColorQueue;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.BezierCurveTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
@Autonomous(name = "Kids Auto", group = "AAA")
public class KidsAuto extends OpMode {
    public RobotController robot;
    private AutoRecorder recorder;
    public static MainConfig cfg;
    private boolean startAuto = false;

    public static double vp = 0.001;//195;
    public static double vs = 0.18;//195;
    public static double vd = 0;//25;
    public static double vf = 0.00032; //0.0004
    public static double angleOffset = 50;
    public double distanceToWallOdometry = 0;
    public static double kVTurret = 0.003;
    public static double kATurret = 0.00015;
    public static double kSTurret = 0.06;
    public double rotationToWallOdometry = 0;

    @Override
    public void init() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                if(startAuto) {
                    startAuto = false;
                    makeAutoKids();
                    distanceToWallOdometry = calculateDistanceToWallInMeters(robot.getCurrentPose(), cfg.usedTargetX, cfg.usedTargetY);
                }
            }
        };
    }

    @Override
    public void start() {
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

    public void makeAutoKids(){
        robot.executeNow(new ActionSequence(
                new GeneralAction(() -> move_backward(20))
        ));
    }

    public void move_forward(double dist){
        dist /= 0.39;
        double new_x = robot.getCurrentPose().getX() + dist * cos(toRadians(robot.getCurrentPose().getHeading()));
        double new_y = robot.getCurrentPose().getY() + dist * sin(toRadians(robot.getCurrentPose().getHeading()));

        robot.addToQueue(new MoveAction(robot.getCurrentPose().withX(new_x).withY(new_y)));
    }

    public void move_backward(double dist) {
        dist /= 0.39;
        double new_x = robot.getCurrentPose().getX() - dist * cos(toRadians(robot.getCurrentPose().getHeading()));
        double new_y = robot.getCurrentPose().getY() - dist * sin(toRadians(robot.getCurrentPose().getHeading()));

        robot.addToQueue(new MoveAction(robot.getCurrentPose().withX(new_x).withY(new_y)));
    }

    public void turn_right(double angle) {
        robot.addToQueue(new MoveAction(robot.getCurrentPose().withHeading(robot.getCurrentPose().getHeading() + angle)));
    }

    public void turn_left(double angle) {
        robot.addToQueue(new MoveAction(robot.getCurrentPose().withHeading(robot.getCurrentPose().getHeading() - angle)));
    }

    public void start_intake() {
        robot.addToQueue(new ActionSequence(
            new StateAction("coupleServo", "DECOUPLED"),
            new StateAction("IntakeMotor", "FULL")
        ));
    }

    public void stop_intake() {
        robot.addToQueue(new StateAction("IntakeMotor", "OFF"));
    }

    public void shoot() {
        shooting_turret();
        robot.executeNow(new ActionSequence(
                new DelayAction(2000), // safe delay for turret to prepare
                new StateAction("coupleServo", "COUPLED"),
                new StateAction("RightGateServo", "OPEN"),
                new DelayAction(timerToCloseGate),
                new StateAction("RightGateServo", "CLOSED"),
                //new DelayAction(timer1),
                new StateAction("LeftGateServo", "OPEN"),
                new DelayAction(timer2),
                new StateAction("RightGateServo", "OPEN"),//

                // close gates
                new DelayAction(600),
                new StateAction("RightGateServo", "CLOSED"),
                new StateAction("LeftGateServo", "CLOSED"),
                new StateAction("IntakeMotor","FULL_REVERSE"),
                new DelayAction(100),
                new StateAction("IntakeMotor","FULL"),
                new StateAction("coupleServo", "DECOUPLED")
        ));
    }

    public void delay(double delay) {
        robot.executeNow(new DelayAction(delay));
    }

    public void shooting_turret() {
        double usedDistance = distanceToWallOdometry;
        // ----------------------- Power Stuff -----------------------

        double targetVelocity = distanceToVelocityFunction(usedDistance) * vMultiplier;
        if(RobotController.currentVoltage < 10 || RobotController.getDrivetrainCumulativePower() >3.85) targetVelocity += 30;
//            if(shouldToAirSort) targetVelocity = airSortingFunctionVelocity(usedDistance) *vMultiplier + D2_velocityAdder
        robot.getMotorComponent("TurretSpinMotor")
                .setAccelerationVelocityCoefficients(vp,0,vd,vf,vs)
                .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                .setTarget(targetVelocity);

        double turretAngleVal = 0;
//            turretAngleVal = distanceToAngleFunction(usedDistance) - (angleOffset * (usedDistance >= 2.9 ? 1 : 0));
        turretAngleVal = distanceToAngleFunction(usedDistance) - (angleOffset * (usedDistance >= 2.9 ? 1 : 0));
//            if(shouldToAirSort) turretAngleVal = airSortingFunctionAngle(usedDistance);
        turretAngleVal = clamp(turretAngleVal,18, 324); // fresh measured
        robot.getServoComponent("TurretAngle")
                .setTarget(turretAngleVal);


        // ----------------------- Rotation Stuff -----------------------
        TurretComponent tempTurret = robot.getTurretComponent("TurretRotateMotor");


//        double usedDistance = 0;
        double neededAngleForTurretRotation = 0;
        usedDistance = distanceToWallOdometry;

        rotationToWallOdometry = tempTurret.calculateLookaheadTarget(usedTargetX, usedTargetY, 0);

        neededAngleForTurretRotation -= rotationToWallOdometry; /// TODO this might be to be reversed in some ways


        if (usedDistance > 2.9) neededAngleForTurretRotation += cfg.farZoneCameraAdder;

        //if(neededAngleForTurretRotation > -30) neededAngleForTurretRotation += rightSideAngleBias;
        if (neededAngleForTurretRotation < 0) neededAngleForTurretRotation += 360;



        // Predict target (Target is at 0,0 in world space for example)
        // lookaheadSeconds should roughly match your control loop latency + motor response time
        tempTurret.setFeedforwardCoefficients(kVTurret,kATurret,kSTurret);
        tempTurret.setOperationMode(MotorComponent.MotorModes.Position);
        tempTurret.setTarget(neededAngleForTurretRotation);
    }
}