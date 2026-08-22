//package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.photon.PhotonCore;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.List;
//
//@Config
//@TeleOp(name = "Hardware Write Queue Stress Test", group = "tests")
//public class HardwareWriteStressTest extends LinearOpMode {
//
//    // --- BENCHMARK CONFIGURATION ---
//    public static int targetWritesPerLoop = 1;      // Starting redundancy count per frame
//    public static int autoIncrementStep = 2;          // Writes added per step in AUTO mode
//    public static double autoStepIntervalSec = 2.0;  // How often to step up writes in AUTO mode
//    public static boolean autoTestMode = true;        // Auto-ramp vs. Manual Gamepad Control
//
//    // --- SATURATION THRESHOLDS ---
//    public static double MAX_ALLOWED_AVG_LOOP_MS = 15.0; // Loop delay threshold for queue saturation
//    public static double MAX_ALLOWED_SPIKE_MS = 30.0;    // Single frame spike threshold
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize PhotonCore / Hardware Caching
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//        PhotonCore.PARALLELIZE_SERVOS = true;
//        PhotonCore.enable();
//
//        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Hardware Bindings
//        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turretFlyWheelMotorRight");
//        Servo servo = hardwareMap.get(Servo.class, "leftTiltServo");
//
//        // Metrics Tracking
//        long lastFrameNanos = System.nanoTime();
//        ElapsedTime stepTimer = new ElapsedTime();
//        ElapsedTime windowTimer = new ElapsedTime();
//
//        double totalLoopMls = 0;
//        double maxLoopMls = 0;
//        int loopCount = 0;
//
//        int safeMaxWritesPerLoop = 0;
//        double safeMaxWritesPerSec = 0;
//        boolean saturationDetected = false;
//
//        // Button Debounce
//        boolean lastDpadUp = false;
//        boolean lastDpadDown = false;
//
//        waitForStart();
//        stepTimer.reset();
//        windowTimer.reset();
//        lastFrameNanos = System.nanoTime();
//
//        while (opModeIsActive()) {
//            // Clear Bulk Caches
//            for (LynxModule hub : allHubs) {
//                hub.clearBulkCache();
//            }
//
//            // 1. Precise Loop Timing
//            long currentNanos = System.nanoTime();
//            double currentLoopMls = (currentNanos - lastFrameNanos) / 1_000_000.0;
//            lastFrameNanos = currentNanos;
//
//            totalLoopMls += currentLoopMls;
//            maxLoopMls = Math.max(maxLoopMls, currentLoopMls);
//            loopCount++;
//
//            // 2. Manual Tuning via Gamepad
//            if (gamepad1.dpad_up && !lastDpadUp) targetWritesPerLoop += 2;
//            if (gamepad1.dpad_down && !lastDpadDown) targetWritesPerLoop = Math.max(1, targetWritesPerLoop - 2);
//            lastDpadUp = gamepad1.dpad_up;
//            lastDpadDown = gamepad1.dpad_down;
//
//            // 3. Automated Ramp-up Logic
//            if (autoTestMode && !saturationDetected && stepTimer.seconds() >= autoStepIntervalSec) {
//                targetWritesPerLoop += autoIncrementStep;
//                stepTimer.reset();
//            }
//
//            // 4. EXECUTE REDUNDANT HARDWARE WRITES
//            // Alternates power values slightly to bypass write-deduplication checks
//            for (int i = 0; i < targetWritesPerLoop; i++) {
//                double val = (i % 2 == 0) ? 0.001 : 0.002;
//                motor.setPower(val);
//                servo.setPosition(0.5 + (val * 10));
//            }
//
//            // 5. Diagnostics and Saturation Analysis (Every 100ms Window)
//            if (windowTimer.milliseconds() >= 100) {
//                double avgLoopMls = totalLoopMls / Math.max(1, loopCount);
//                double currentHz = loopCount * (1000.0 / windowTimer.milliseconds());
//                double writesPerSecond = targetWritesPerLoop * currentHz;
//
//                // Check saturation criteria
//                if (avgLoopMls > MAX_ALLOWED_AVG_LOOP_MS || maxLoopMls > MAX_ALLOWED_SPIKE_MS) {
//                    saturationDetected = true;
//                } else if (!saturationDetected) {
//                    safeMaxWritesPerLoop = targetWritesPerLoop;
//                    safeMaxWritesPerSec = writesPerSecond;
//                }
//
//                // Telemetry Display
//                tel.addData("--- STATUS ---", saturationDetected ? "⚠️ QUEUE SATURATED / OVERFLOW" : "✅ CLEAN QUEUE");
//                tel.addData("Writes Per Loop", targetWritesPerLoop);
//                tel.addData("Writes Per Second", "%.0f writes/sec", writesPerSecond);
//                tel.addData("Current Frame Rate", "%.1f Hz", currentHz);
//                tel.addData("Avg Loop Time", "%.2f ms (Limit: %.1f ms)", avgLoopMls, MAX_ALLOWED_AVG_LOOP_MS);
//                tel.addData("Max Loop Spike", "%.2f ms (Limit: %.1f ms)", maxLoopMls, MAX_ALLOWED_SPIKE_MS);
//                tel.addLine("----------------------------------");
//                tel.addData("SAFE MAX WRITES/LOOP", safeMaxWritesPerLoop);
//                tel.addData("SAFE MAX WRITES/SEC", "%.0f writes/sec", safeMaxWritesPerSec);
//                tel.addLine("Controls: D-Pad Up/Down to manually tune write count.");
//                tel.update();
//
//                // Reset Window Metrics
//                totalLoopMls = 0;
//                maxLoopMls = 0;
//                loopCount = 0;
//                windowTimer.reset();
//            }
//        }
//    }
//}