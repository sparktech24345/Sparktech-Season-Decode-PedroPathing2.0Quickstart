package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.anygeneric.blazeftc.DummyPlugOpMode

@TeleOp(name = "NeutrinoTest")
class NeutrinoTest : DummyPlugOpMode() {
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
        // Pass standard FTC telemetry directly
        initializeBlazeFTC(telemetry)

        telemetry.addData("Status", "BlazeFTC Initialized Successfully!")
        telemetry.update()

        waitForStart()

        runBlazeFTC(0) // Start default Neutrino proxy

        val time = ElapsedTime()
        while (!isStopRequested) {
            val ms = time.milliseconds()
            telemetry.addData("Java loop time", "${ms - millisToWait}[ms]")
            telemetry.update()
            time.reset()
            Thread.sleep(millisToWait, nanosToWait)
        }
    }
}