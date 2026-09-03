package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import dev.anygeneric.blazeftc.DummyPlugOpMode
import dev.anygeneric.blazeftc.PositionData
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.concurrent.Volatile
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "Test all motors Kotlin")
class testAllMotorsKotlin : DummyPlugOpMode() {

    var servoPos: Double = 0.88
    var motorPowe: Double = 0.0

    // Color Sensor State
    protected var colorSensorRight: NormalizedColorSensor? = null
    protected var colorSensorLeft: NormalizedColorSensor? = null

    var rightColors: NormalizedRGBA? = null
    var leftColors: NormalizedRGBA? = null
    var pinpointData: PositionData? = null

    // Async Pinpoint State
    @Volatile
    private var latestPinpointPos: PositionData? = null

    // Hardware Handles
    private var motor1: DcMotorEx? = null;  // Hardware Handles
    private var motor2: DcMotorEx? = null;  // Hardware Handles
    private var motor3: DcMotorEx? = null;  // Hardware Handles
    private var motor4: DcMotorEx? = null;  // Hardware Handles
    private var motor5: DcMotorEx? = null;  // Hardware Handles
    private var motor6: DcMotorEx? = null;  // Hardware Handles
    private var motor7: DcMotorEx? = null;  // Hardware Handles
    private var motor8: DcMotorEx? = null
    private var leftTiltServo: Servo? = null;
    private var rightTiltServo: Servo? = null;
    private var rightGateServo: Servo? = null
    private var leftGateServo: Servo? = null
    private var turretAngleServo: Servo? = null;
    private var CameraRotateServo: Servo? = null;
    private var coupleServo: Servo? = null
    private var limelight: Limelight3A? = null
    private var pinpoint: GoBildaPinpointDriver? = null

    // Background Executors
    private var colorSensorExecutor: ScheduledExecutorService? = null
    private var telExecutor: ExecutorService? = null

    // Loop Measurement
    private var lastFrameTimeNanos: Long = 0
    private var maxLoopMls = 0.0
    private var minLoopMls = Double.Companion.MAX_VALUE
    private var totalLoopMls = 0.0
    private var loopCount = 0


    override fun runOpModeInBlaze() {
        TODO("Not yet implemented")
    }

    companion object {
        @JvmStatic
        var millisToWait = 5L
        @JvmStatic
        var nanosToWait = 0
    }

    override fun runOpMode() {
        for (i in hardwareMap.getAll<LynxModule?>(LynxModule::class.java)) i.setBulkCachingMode(
            LynxModule.BulkCachingMode.MANUAL
        )


        // 1. Initialize Hardware Map Objects First
        motor1 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "turretFlyWheelMotorRight")
        motor2 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "turretFlyWheelMotorLeft")
        motor3 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "turretRotateMotor")
        motor4 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "intakeMotor")
        motor5 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "frontright")
        motor6 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "frontleft")
        motor7 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "backleft")
        motor8 = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "backright")

        leftTiltServo = hardwareMap.get<Servo?>(Servo::class.java, GlobalStorage.leftTiltServoName)
        rightTiltServo =
            hardwareMap.get<Servo?>(Servo::class.java, GlobalStorage.rightTiltServoName)
        rightGateServo =
            hardwareMap.get<Servo?>(Servo::class.java, GlobalStorage.rightGateServoName)
        leftGateServo = hardwareMap.get<Servo?>(Servo::class.java, GlobalStorage.leftGateServoName)
        turretAngleServo =
            hardwareMap.get<Servo?>(Servo::class.java, GlobalStorage.turretAngleServoName)
        CameraRotateServo =
            hardwareMap.get<Servo?>(Servo::class.java, GlobalStorage.CameraRotateServoName)
        coupleServo = hardwareMap.get<Servo?>(Servo::class.java, GlobalStorage.coupleServoName)

        colorSensorRight = hardwareMap.get<NormalizedColorSensor?>(
            NormalizedColorSensor::class.java,
            GlobalStorage.colorSensorRightName
        )
        colorSensorLeft = hardwareMap.get<NormalizedColorSensor?>(
            NormalizedColorSensor::class.java,
            GlobalStorage.colorSensorLeftName
        )
        limelight = hardwareMap.get<Limelight3A?>(Limelight3A::class.java, "limelight")

        pinpoint =
            hardwareMap.get<GoBildaPinpointDriver?>(GoBildaPinpointDriver::class.java, "pinpoint")
        pinpoint!!.setOffsets(-116.5, -119.38, DistanceUnit.MM)
        pinpoint!!.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint!!.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )
        pinpoint!!.setYawScalar(1.0)
        pinpoint!!.resetPosAndIMU()


        // 2. Wrap registered motors for write acceleration
        engageMotorAcceleration()


        // 4. Hook underlying Lynx streams & create cached telemetry
//        Telemetry blazeTelemetry = initializeBlazeFTC(this.telemetry);
        val tel: Telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())

        telExecutor = Executors.newSingleThreadExecutor()
        colorSensorExecutor = Executors.newSingleThreadScheduledExecutor()

        waitForStart()


        // 5. Start Neutrino native proxy mode AFTER start
        runBlazeFTC(0)

        if (isStopRequested()) return



        colorSensorExecutor!!.scheduleWithFixedDelay(Runnable {
            if (opModeIsActive()) {
                if (colorSensorRight != null) rightColors = colorSensorRight!!.getNormalizedColors()
                if (colorSensorLeft != null) leftColors = colorSensorLeft!!.getNormalizedColors()
            }
        }, 0, 20, TimeUnit.MILLISECONDS)

        lastFrameTimeNanos = System.nanoTime()
        val telTimer = ElapsedTime()


        while (opModeIsActive()) {
            try {
                for (i in hardwareMap.getAll<LynxModule?>(LynxModule::class.java)) {
                    i.clearBulkCache()
                }

                // Loop Timing
                val currentNanos = System.nanoTime()
                val currentLoopMls = (currentNanos - lastFrameTimeNanos) / 1000000.0
                lastFrameTimeNanos = currentNanos

                maxLoopMls = max(maxLoopMls, currentLoopMls)
                minLoopMls = min(minLoopMls, currentLoopMls)
                totalLoopMls += currentLoopMls
                loopCount++

                pinpointData = latestPinpointPos

                // Actuation
                motor1!!.setPower(TestAllMotors.motorPowe)
                motor2!!.setPower(TestAllMotors.motorPowe)
                motor3!!.setPower(TestAllMotors.motorPowe)
                motor4!!.setPower(TestAllMotors.motorPowe)

                //                motor5.setPower(motorPowe);
                if (leftTiltServo != null) leftTiltServo!!.setPosition(ServoMultiple0s.leftTiltPos)
                if (rightTiltServo != null) rightTiltServo!!.setPosition(ServoMultiple0s.rightTiltPos)
                if (rightGateServo != null) rightGateServo!!.setPosition(ServoMultiple0s.rightGatePos)
                if (leftGateServo != null) leftGateServo!!.setPosition(ServoMultiple0s.leftGatePos)
                if (CameraRotateServo != null) CameraRotateServo!!.setPosition(ServoMultiple0s.CameraServoPos)
                if (turretAngleServo != null) turretAngleServo!!.setPosition(ServoMultiple0s.angleServoPos)
                if (coupleServo != null) coupleServo!!.setPosition(ServoMultiple0s.coupleServoPos)

                motor1!!.getCurrentPosition()
                motor2!!.getCurrentPosition()
                motor3!!.getCurrentPosition()
                motor4!!.getCurrentPosition()
                motor5!!.getCurrentPosition()
                motor6!!.getCurrentPosition()
                motor7!!.getCurrentPosition()
                motor8!!.getCurrentPosition()

                motor1!!.getVelocity()
                motor2!!.getVelocity()
                motor3!!.getVelocity()
                motor4!!.getVelocity()
                motor5!!.getVelocity()
                motor6!!.getVelocity()
                motor7!!.getVelocity()
                motor8!!.getVelocity()

                // --- TELEMETRY AGGREGATION (Capped at ~30 Hz / 33 ms) ---
                if (telTimer.milliseconds() >= 33) {
                    val elapsedTelMls = telTimer.milliseconds()

                    val snapshotMax = maxLoopMls
                    val snapshotMin = minLoopMls
                    val snapshotAvg = totalLoopMls / max(1, loopCount)
                    val actualHz = loopCount * (1000.0 / elapsedTelMls)
                    val snapshotLoopCount = loopCount

                    telExecutor!!.submit(Runnable {
                        tel.addData("Actual Hz (Loops/Sec)", "%.1f", actualHz)
                        tel.addData("Avg Loop Time (ms)", "%.2f", snapshotAvg)
                        tel.addData("Min Loop Time (ms)", "%.2f", snapshotMin)
                        tel.addData("Max Loop Spike (ms)", "%.2f", snapshotMax)
                        tel.addData("Loops in 33ms Window", snapshotLoopCount)

                        if (rightColors != null) tel.addData("Right Red", "%.3f", rightColors!!.red)
                        if (leftColors != null) tel.addData("Left Red", "%.3f", leftColors!!.red)
                        if (pinpointData != null) tel.addData("Pinpoint Data", pinpointData!!.xPosition)
                        tel.update()
                    })

                    maxLoopMls = 0.0
                    minLoopMls = Double.Companion.MAX_VALUE
                    totalLoopMls = 0.0
                    loopCount = 0
                    telTimer.reset()
                }

                // Throttle Java loop to prevent JNI buffer saturation/crashes
                try {
                    Thread.sleep(3)
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                }
            } catch (e: Exception) {
                TODO("Not yet implemented")
            }
        }
        colorSensorExecutor!!.shutdownNow()
        telExecutor!!.shutdownNow()
    }
}