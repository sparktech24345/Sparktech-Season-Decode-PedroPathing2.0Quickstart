package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static dev.anygeneric.blazeftc.BlazeDummyPlug.initializeBlazeFTC;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import dev.anygeneric.blazeftc.BlazeFTC;
import dev.anygeneric.blazeftc.DummyPlugOpMode;

@TeleOp(name = "BlazeJavaMecanum")
public class BlazeJavaMecanum extends DummyPlugOpMode {

    // Hardware motor declarations
    private DcMotor RFDrive = null;
    private DcMotor LFDrive = null;
    private DcMotor RBDrive = null;
    private DcMotor LBDrive = null;

    // Configuration settings from your original DriveTrain class
    private boolean directionFlip = false;
    private double slowdownMultiplier = 1.0;
    private double drivetrainCumulativePower = 0;

    // Default configuration names from your snippet
    private final String frontLeftName  = "frontLeftName";
    private final String frontRightName = "frontRightName";
    private final String backLeftName   = "backLeftName";
    private final String backRightName  = "backRightName";

    @Override
    public void runOpModeInBlaze() {


        initializeBlazeFTC();
        engageMotorAcceleration();

        // Standard hardware map initialization
        LFDrive = hardwareMap.get(DcMotor.class, frontLeftName);
        RFDrive = hardwareMap.get(DcMotor.class, frontRightName);
        LBDrive = hardwareMap.get(DcMotor.class, backLeftName);
        RBDrive = hardwareMap.get(DcMotor.class, backRightName);

        // Typical Mecanum motor orientation (adjust if your motors push backwards)
        LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero power behavior setup
        LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        runBlazeFTC(0);

        ElapsedTime elt2 = new ElapsedTime();

        while (!isStopRequested()) {
            for (LynxModule i : hardwareMap.getAll(LynxModule.class)) {
                i.clearBulkCache();
            }

            // --- INPUT HANDLING ---
            // Standard Mecanum drive vectors from gamepad sticks
            double y = -gamepad1.left_stick_y; // Y stick value is reversed by default
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Dynamic slowdown toggle (Example: Left bumper triggers slow mode)
            if (gamepad1.left_bumper) {
                slowdownMultiplier = 0.5;
            } else {
                slowdownMultiplier = 1.0;
            }

            // Direction flipping mechanics from your original file
            if (gamepad1.y) {
                directionFlip = true;   // Invert robot front/back alignment
            } else if (gamepad1.a) {
                directionFlip = false;  // Reset to default alignment
            }

            // Apply orientation corrections
            if (directionFlip) {
                y = -y;
                x = -x;
            }

            // --- MECANUM DIRECT MOTOR MATH ---
            // Normalize values so they never exceed 1.0 power boundary
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // Apply global slowdown multiplier
            frontLeftPower  *= slowdownMultiplier;
            backLeftPower   *= slowdownMultiplier;
            frontRightPower *= slowdownMultiplier;
            backRightPower  *= slowdownMultiplier;

            // --- MOTOR POWER OUTPUT ---
//            LFDrive.setPower(frontLeftPower);
//            LBDrive.setPower(backLeftPower);
//            RFDrive.setPower(frontRightPower);
//            RBDrive.setPower(backRightPower);

            BlazeFTC.setMotorPower(173,LFDrive.getPortNumber(),frontLeftPower);
            BlazeFTC.setMotorPower(173,LBDrive.getPortNumber(),backLeftPower);
            BlazeFTC.setMotorPower(173,RFDrive.getPortNumber(),frontRightPower);
            BlazeFTC.setMotorPower(173,RBDrive.getPortNumber(),backRightPower);

            // Calculate overall cumulative power metric based on motor levels
            drivetrainCumulativePower = Math.abs(frontLeftPower) + Math.abs(frontRightPower) +
                    Math.abs(backLeftPower) + Math.abs(backRightPower);

            // --- TELEMETRY DATA ---
            telemetry.addData("main loop time (ms)", elt2.milliseconds());
            elt2.reset();

            telemetry.addLine("--- Drivetrain Status ---");
            telemetry.addData("Slowdown Multiplier", slowdownMultiplier);
            telemetry.addData("Direction Flipped", directionFlip);
            telemetry.addData("Cumulative Power", drivetrainCumulativePower);
            telemetry.addData("Motor Powers", "LF: %.2f | RF: %.2f | LB: %.2f | RB: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            telemetry.update();

            // Small safety sleep to match your template base
            sleep(20);
        }
    }
}
