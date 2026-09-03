package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.ServoMultiple0s.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import dev.anygeneric.blazeftc.DummyPlugOpMode;

@Config
@TeleOp(name = "Test all motors and loop time", group = "tests")
public class TestAllMotors extends DummyPlugOpMode {
    public static double motorPowe = 0;

    protected NormalizedColorSensor colorSensorRight, colorSensorLeft;
    private NormalizedRGBA rightColors, leftColors;

    private DcMotorEx motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8;
    private Servo leftTiltServo, rightTiltServo, rightGateServo, leftGateServo;
    private Servo turretAngleServo, CameraRotateServo, coupleServo;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    private ScheduledExecutorService colorSensorExecutor;

    private long lastFrameTimeNanos = 0;
    private double maxLoopMls = 0;
    private double minLoopMls = Double.MAX_VALUE;
    private double totalLoopMls = 0;
    private int loopCount = 0;

    @Override
    public void runOpModeInBlaze() {

        motor1 = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");
        motor2 = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        motor3 = hardwareMap.get(DcMotorEx.class, "turretRotateMotor");
        motor4 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motor5 = hardwareMap.get(DcMotorEx.class, "frontright");
        motor6 = hardwareMap.get(DcMotorEx.class, "frontleft");
        motor7 = hardwareMap.get(DcMotorEx.class, "backleft");
        motor8 = hardwareMap.get(DcMotorEx.class, "backright");

        leftTiltServo = hardwareMap.get(Servo.class, leftTiltServoName);
        rightTiltServo = hardwareMap.get(Servo.class, rightTiltServoName);
        rightGateServo = hardwareMap.get(Servo.class, rightGateServoName);
        leftGateServo = hardwareMap.get(Servo.class, leftGateServoName);
        turretAngleServo = hardwareMap.get(Servo.class, turretAngleServoName);
        CameraRotateServo = hardwareMap.get(Servo.class, CameraRotateServoName);
        coupleServo = hardwareMap.get(Servo.class, coupleServoName);

        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-116.5, -119.38, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        engageMotorAcceleration();

        initializeBlazeFTC(this.telemetry);
        Telemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        colorSensorExecutor = Executors.newSingleThreadScheduledExecutor();

        waitForStart();

        runBlazeFTC(0); // Start Neutrino handler

        if (isStopRequested()) return;

        colorSensorExecutor.scheduleWithFixedDelay(() -> {
            if (opModeIsActive()) {
                if (colorSensorRight != null) rightColors = colorSensorRight.getNormalizedColors();
                if (colorSensorLeft != null) leftColors = colorSensorLeft.getNormalizedColors();
            }
        }, 0, 20, TimeUnit.MILLISECONDS);

        lastFrameTimeNanos = System.nanoTime();
        ElapsedTime telTimer = new ElapsedTime();

        while (opModeIsActive()) {
            long currentNanos = System.nanoTime();
            double currentLoopMls = (currentNanos - lastFrameTimeNanos) / 1_000_000.0;
            lastFrameTimeNanos = currentNanos;

            maxLoopMls = Math.max(maxLoopMls, currentLoopMls);
            minLoopMls = Math.min(minLoopMls, currentLoopMls);
            totalLoopMls += currentLoopMls;
            loopCount++;

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            motor1.setPower(motorPowe);
            motor2.setPower(motorPowe);
            motor3.setPower(motorPowe);
            motor4.setPower(motorPowe);

            if (leftTiltServo != null) leftTiltServo.setPosition(leftTiltPos);
            if (rightTiltServo != null) rightTiltServo.setPosition(rightTiltPos);
            if (rightGateServo != null) rightGateServo.setPosition(rightGatePos);
            if (leftGateServo != null) leftGateServo.setPosition(leftGatePos);
            if (CameraRotateServo != null) CameraRotateServo.setPosition(CameraServoPos);
            if (turretAngleServo != null) turretAngleServo.setPosition(angleServoPos);
            if (coupleServo != null) coupleServo.setPosition(coupleServoPos);

            // Telemetry updated safely on main thread (~30 Hz)
            if (telTimer.milliseconds() >= 33) {
                double elapsedMls = telTimer.milliseconds();
                tel.addData("Actual Hz", "%.1f", loopCount * (1000.0 / elapsedMls));
                tel.addData("Avg Loop (ms)", "%.2f", totalLoopMls / Math.max(1, loopCount));
                tel.addData("Min Loop (ms)", "%.2f", minLoopMls);
                tel.addData("Max Spike (ms)", "%.2f", maxLoopMls);

                if (rightColors != null) tel.addData("Right Red", "%.3f", rightColors.red);
                if (pose != null) tel.addData("Pinpoint X", "%.2f", pose.getX(DistanceUnit.MM));

                tel.update();

                maxLoopMls = 0;
                minLoopMls = Double.MAX_VALUE;
                totalLoopMls = 0;
                loopCount = 0;
                telTimer.reset();
            }

            try {
                Thread.sleep(3);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        colorSensorExecutor.shutdownNow();
    }
}