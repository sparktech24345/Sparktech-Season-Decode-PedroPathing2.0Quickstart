package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class DriveTrain {

    private static DcMotor RFDrive;
    private static DcMotor LFDrive;
    private static DcMotor RBDrive;
    private static DcMotor LBDrive;
    private static boolean directionFlip = false;
    private static String frontLeft  = frontLeftName;
    private static String frontRight = frontRightName;
    private static String backLeft   = backLeftName;
    private static String backRight  = backRightName;
    private static double slowdownMultiplier = 1;
    private static boolean init_ = false;


    public static void init(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        frontLeft = LeftFront;
        frontRight = RightFront;
        backLeft = LeftBack;
        backRight = RightBack;
        init();
    }

    public static boolean wasInitialized() { return init_; }

    public static void setSlowdown(double slowdownMultiplier) {
        DriveTrain.slowdownMultiplier = slowdownMultiplier;
    }
    public static void setDirectionFlip(boolean shouldFlip) {
        DriveTrain.directionFlip = shouldFlip;
    }
    public static boolean getDirectionFlip() {
        return DriveTrain.directionFlip;
    }

    public static void init() {
        if (currentOpModes == OpModes.TeleOP) {
            init_ = true;
            RFDrive = hardwareMap.get(DcMotor.class, frontRight);
            LFDrive = hardwareMap.get(DcMotor.class, frontLeft);
            RBDrive = hardwareMap.get(DcMotor.class, backRight);
            LBDrive = hardwareMap.get(DcMotor.class, backLeft);

            RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public static void loop() {

        double vertical     = -ComplexGamepad.get("LEFT_STICK_Y1").raw();
        double horizontal   = -ComplexGamepad.get("LEFT_STICK_X1").raw();
        double pivot        = ComplexGamepad.get("RIGHT_STICK_X1").raw();

        if (directionFlip) {
            horizontal = - horizontal;
            vertical = - vertical;
        }

        double FrontRightPow = vertical + horizontal - pivot;
        double BackRightPow  = vertical - horizontal - pivot;
        double FrontLeftPow  = vertical - horizontal + pivot;
        double BackLeftPow   = vertical + horizontal + pivot;

        RFDrive.setPower(FrontRightPow * slowdownMultiplier);
        LFDrive.setPower(FrontLeftPow * slowdownMultiplier);
        RBDrive.setPower(BackRightPow * slowdownMultiplier);
        LBDrive.setPower(BackLeftPow * slowdownMultiplier);
    }

    public void telemetry() {}
}
