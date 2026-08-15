package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import com.seattlesolvers.solverslib.photon.PhotonCore;

@Config
@TeleOp(name = "Test all motors and loop time", group = "Linear OpMode")
public class TestAllMotors extends LinearOpMode {
    public static double servoPos = 0.88;
    public static double motorPowe = 0;

    /// ----------------- Color Sensor Stuff ------------------
    protected NormalizedColorSensor colorSensorRight;
    protected NormalizedColorSensor colorSensorLeft;
    protected NormalizedRGBA rightSensorColors;
    protected NormalizedRGBA leftSensorColors;

    public static int ballCounter = 0;
    protected BallColorSet_Decode actualRightSensorDetectedBall;
    protected BallColorSet_Decode calculatedRightSensorDetectedBall;
    protected BallColorSet_Decode actualLeftSensorDetectedBall;
    protected BallColorSet_Decode calculatedLeftSensorDetectedBall;
    protected BallColorSet_Decode ballToFire;
    public static boolean hasBallInIntake = false;
    public static boolean hasBallInRightChamber = false;
    public static boolean hasBallInLeftChamber = false;
    public static boolean shouldRemoveBalls = false;
    public static double OuttakePIDSwitch = 81;
    public static boolean shouldPullFromQueue = false;
    public static boolean wantsToTempOutputIntake = false;
    public static boolean shouldResetRightSensorBall = false;
    public static boolean ShouldSpewOutSensors = false;
    public static boolean shouldSpewCameraTelemetry = false;
    public static boolean disableTurret = false;

    // Cache to prevent redundant motor commands across loops
    private double lastMotorPower = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        // Bulk caching setup
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8); // Can be adjusted based on user preference - but raising this number further can cause issues

        //snth
//         REMOVED setMaximumParallelCommands(8) to prevent RS-485 serial timeouts / packet drop spikes
        PhotonCore.PARALLELIZE_SERVOS = true;
        PhotonCore.enable();

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime telTimer = new ElapsedTime();

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorLeft");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "turretRotateMotor");
        DcMotorEx motor4 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx motor5 = hardwareMap.get(DcMotorEx.class, "frontright");
        DcMotorEx motor6 = hardwareMap.get(DcMotorEx.class, "frontleft");
        DcMotorEx motor7 = hardwareMap.get(DcMotorEx.class, "backleft");
        DcMotorEx motor8 = hardwareMap.get(DcMotorEx.class, "backright");

        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, colorSensorRightName);
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, colorSensorLeftName);

        waitForStart();
        if (isStopRequested()) return;

        loopTimer.reset();
        telTimer.reset();

        double maxLoopMls = 0;
        double totalLoopMls = 0;
        int loopCount = 0;

        while (opModeIsActive()) {
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();

            // 1. Calculate current frame time
            double lastLoopMls = loopTimer.milliseconds();
            loopTimer.reset();

            // 2. Accumulate window statistics
            maxLoopMls = Math.max(maxLoopMls, lastLoopMls);
            totalLoopMls += lastLoopMls;
            loopCount++;

            motor1.setPower(motorPowe);
            motor2.setPower(motorPowe);
            motor3.setPower(motorPowe);
            motor4.setPower(motorPowe);
            motor5.setPower(motorPowe);
            motor6.setPower(motorPowe);
            motor7.setPower(motorPowe);
            motor8.setPower(motorPowe);
            motor1.getCurrent(CurrentUnit.AMPS);
            motor2.getCurrent(CurrentUnit.AMPS);
            motor3.getCurrent(CurrentUnit.AMPS);
            motor4.getCurrent(CurrentUnit.AMPS);
            motor5.getCurrent(CurrentUnit.AMPS);
            motor6.getCurrent(CurrentUnit.AMPS);
            motor7.getCurrent(CurrentUnit.AMPS);
            motor8.getCurrent(CurrentUnit.AMPS);
            motor1.getVelocity();
            motor2.getVelocity();
            motor3.getVelocity();
            motor4.getVelocity();
            motor5.getVelocity();
            motor6.getVelocity();
            motor7.getVelocity();
            motor8.getVelocity();
            motor1.getCurrentPosition();
            motor2.getCurrentPosition();
            motor3.getCurrentPosition();
            motor4.getCurrentPosition();
            motor5.getCurrentPosition();
            motor6.getCurrentPosition();
            motor7.getCurrentPosition();
            motor8.getCurrentPosition();

            // 4. Push MAX and AVG to FTC Dashboard every 33ms
            if (telTimer.milliseconds() >= 33) {
                tel.addData("maxMls", maxLoopMls); // Catches ANY spike in the last 33ms
                tel.addData("avgMls", totalLoopMls / loopCount); // True average speed
                tel.addData("hz", loopCount * (1000.0 / telTimer.milliseconds())); // True frequency
                tel.update();

                // Reset accumulation trackers for next 33ms window
                maxLoopMls = 0;
                totalLoopMls = 0;
                loopCount = 0;
                telTimer.reset();
            }
        }
    }
}