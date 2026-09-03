package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.anygeneric.blazeftc.DummyPlugOpMode

@TeleOp(name = "Minimal Blaze Test", group = "tests")
class MinimalBlazeTest : DummyPlugOpMode() {
    private var testMotor: DcMotorEx? = null
    private var testServo: Servo? = null

    override fun runOpModeInBlaze() {
        // 1. Get 1 motor and 1 servo from hardwareMap
        testMotor = hardwareMap.get<DcMotorEx?>(DcMotorEx::class.java, "frontleft")
        testServo = hardwareMap.get<Servo?>(Servo::class.java, "turretRotateMotor")

        // 2. Enable BlazeFTC write acceleration
        engageMotorAcceleration()

        // 3. Initialize BlazeFTC native connection
        initializeBlazeFTC(this.telemetry)

        telemetry.addData("Status", "Initialized. Press START.")
        telemetry.update()

        waitForStart()

        // 4. Start native proxy (id = 0)
        runBlazeFTC(0)

        while (opModeIsActive()) {
            // Control motor with left stick y
            testMotor!!.setPower(-gamepad1.left_stick_y.toDouble())

            // Toggle servo with A / B buttons
            if (gamepad1.a) {
                testServo!!.setPosition(1.0)
            } else if (gamepad1.b) {
                testServo!!.setPosition(0.0)
            }

            telemetry.addData("Status", "Running")
            telemetry.addData("Motor Power", testMotor!!.getPower())
            telemetry.addData("Servo Pos", testServo!!.getPosition())
            telemetry.update()

            try {
                Thread.sleep(5)
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
    }
}