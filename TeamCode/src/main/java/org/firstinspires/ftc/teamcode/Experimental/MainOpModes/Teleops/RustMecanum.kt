package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.anygeneric.blazeftc.DummyPlugOpMode

@TeleOp(name = "Mecanum Rust")
//@Configurable //put whatever configuration annotations you need
class RustMecanum : DummyPlugOpMode() {
    override fun runOpModeInBlaze() {
        TODO("Not yet implemented")
    }

    //@Configurable
    companion object {
        @JvmStatic
        var toRun = 1
        @JvmStatic
        var millisToWait = 5L
        @JvmStatic
        var nanosToWait = 0
    }

    override fun runOpMode() {
        //this function can be called after waitForStart but like, don't do that
        //once you've supplied it telemetry, just use the "telemetry" variable it has been set
        initializeBlazeFTC(JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry))

        waitForStart()

        runBlazeFTC(1)//always call this please. toRun is an integer passed into the rust code you can control

        val timer = ElapsedTime()
        while (!isStopRequested) {
            updateGamepads()//*probably* need to call this. depends on what you're doing

            val ms = timer.milliseconds()//timer. feel free to remove this
            timer.reset()
            telemetry.addData("java loop time", "$ms[ms]")
            //the thread sleep is so you don't send so many packets you crash the system
            //there is probably a better way to do it than this but I don't know what it is.
            Thread.sleep(millisToWait, nanosToWait)
        }
    }
}