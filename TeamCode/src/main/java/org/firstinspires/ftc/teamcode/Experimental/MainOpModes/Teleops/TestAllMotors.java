package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.CameraRotateServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.coupleServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftGateServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.leftTiltServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.rightGateServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.rightTiltServoName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.turretAngleServoName;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.ServoMultiple0s.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import dev.anygeneric.blazeftc.DummyPlugOpMode;
import dev.anygeneric.blazeftc.PositionData;
import kotlin.Unit;

@Config
@TeleOp(name = "Test all motors and loop time", group = "tests")
public class TestAllMotors extends DummyPlugOpMode {
    public static double servoPos = 0.88;
    public static double motorPowe = 0;

    // Color Sensor State
    protected NormalizedColorSensor colorSensorRight;
    protected NormalizedColorSensor colorSensorLeft;

    NormalizedRGBA rightColors;
    NormalizedRGBA leftColors;
    PositionData pinpointData;

    // Async Pinpoint State
    private volatile PositionData latestPinpointPos;

    // Hardware Handles
    private DcMotorEx motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8;
    private Servo leftTiltServo, rightTiltServo, rightGateServo, leftGateServo;
    private Servo turretAngleServo, CameraRotateServo, coupleServo;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // Background Executors
    private ScheduledExecutorService colorSensorExecutor;
    private ExecutorService telExecutor;

    // Loop Measurement
    private long lastFrameTimeNanos = 0;
    private double maxLoopMls = 0;
    private double minLoopMls = Double.MAX_VALUE;
    private double totalLoopMls = 0;
    private int loopCount = 0;

    @Override
    public void runOpModeInBlaze() {
        // 1. Initialize Hardware Map Objects First
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
        pinpoint.setYawScalar(1);
        pinpoint.resetPosAndIMU();

        // 2. Wrap registered motors for write acceleration
        engageMotorAcceleration();

        // 3. Register Pinpoint acceleration callback
        engagePinpointAcceleration(pinpoint, true, 2, data -> {
            latestPinpointPos = data;
            return Unit.INSTANCE;
        });

        // 4. Hook underlying Lynx streams & create cached telemetry
        Telemetry blazeTelemetry = initializeBlazeFTC(this.telemetry);
        Telemetry tel = new MultipleTelemetry(blazeTelemetry, FtcDashboard.getInstance().getTelemetry());

        telExecutor = Executors.newSingleThreadExecutor();
        colorSensorExecutor = Executors.newSingleThreadScheduledExecutor();

        waitForStart();
        if (isStopRequested()) return;

        // 5. Start Neutrino native proxy mode AFTER start
        runBlazeFTC(0);

        colorSensorExecutor.scheduleWithFixedDelay(() -> {
            if (opModeIsActive()) {
                if (colorSensorRight != null) rightColors = colorSensorRight.getNormalizedColors();
                if (colorSensorLeft != null) leftColors = colorSensorLeft.getNormalizedColors();
            }
        }, 0, 20, TimeUnit.MILLISECONDS);

        lastFrameTimeNanos = System.nanoTime();
        ElapsedTime telTimer = new ElapsedTime();


            while (opModeIsActive()) {
                for (LynxModule i : hardwareMap.getAll(LynxModule.class)) {
                    i.clearBulkCache();
                }

                // Loop Timing
                long currentNanos = System.nanoTime();
                double currentLoopMls = (currentNanos - lastFrameTimeNanos) / 1_000_000.0;
                lastFrameTimeNanos = currentNanos;

                maxLoopMls = Math.max(maxLoopMls, currentLoopMls);
                minLoopMls = Math.min(minLoopMls, currentLoopMls);
                totalLoopMls += currentLoopMls;
                loopCount++;

                pinpointData = latestPinpointPos;

                // Actuation
                motor1.setPower(motorPowe);
                motor2.setPower(motorPowe);
                motor3.setPower(motorPowe);
                motor4.setPower(motorPowe);
//                motor5.setPower(motorPowe);

                if (leftTiltServo != null) leftTiltServo.setPosition(leftTiltPos);
                if (rightTiltServo != null) rightTiltServo.setPosition(rightTiltPos);
                if (rightGateServo != null) rightGateServo.setPosition(rightGatePos);
                if (leftGateServo != null) leftGateServo.setPosition(leftGatePos);
                if (CameraRotateServo != null) CameraRotateServo.setPosition(CameraServoPos);
                if (turretAngleServo != null) turretAngleServo.setPosition(angleServoPos);
                if (coupleServo != null) coupleServo.setPosition(coupleServoPos);

            motor1.getCurrentPosition();
            motor2.getCurrentPosition();
            motor3.getCurrentPosition();
            motor4.getCurrentPosition();
            motor5.getCurrentPosition();
            motor6.getCurrentPosition();
            motor7.getCurrentPosition();
            motor8.getCurrentPosition();

            motor1.getVelocity();
            motor2.getVelocity();
            motor3.getVelocity();
            motor4.getVelocity();
            motor5.getVelocity();
            motor6.getVelocity();
            motor7.getVelocity();
            motor8.getVelocity();

            // --- TELEMETRY AGGREGATION (Capped at ~30 Hz / 33 ms) ---
            if (telTimer.milliseconds() >= 33) {
                double elapsedTelMls = telTimer.milliseconds();

                    double snapshotMax = maxLoopMls;
                    double snapshotMin = minLoopMls;
                    double snapshotAvg = totalLoopMls / Math.max(1, loopCount);
                    double actualHz = loopCount * (1000.0 / elapsedTelMls);
                    int snapshotLoopCount = loopCount;

                    telExecutor.submit(() -> {
                        tel.addData("Actual Hz (Loops/Sec)", "%.1f", actualHz);
                        tel.addData("Avg Loop Time (ms)", "%.2f", snapshotAvg);
                        tel.addData("Min Loop Time (ms)", "%.2f", snapshotMin);
                        tel.addData("Max Loop Spike (ms)", "%.2f", snapshotMax);
                        tel.addData("Loops in 33ms Window", snapshotLoopCount);

                        if (rightColors != null) tel.addData("Right Red", "%.3f", rightColors.red);
                        if (leftColors != null) tel.addData("Left Red", "%.3f", leftColors.red);
                        if (pinpointData != null) tel.addData("Pinpoint Data", pinpointData.getXPosition());

                        tel.update();
                    });

                    maxLoopMls = 0;
                    minLoopMls = Double.MAX_VALUE;
                    totalLoopMls = 0;
                    loopCount = 0;
                    telTimer.reset();
                }

                // Throttle Java loop to prevent JNI buffer saturation/crashes
                try {
                    Thread.sleep(3);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        colorSensorExecutor.shutdownNow();
        telExecutor.shutdownNow();

    }
}