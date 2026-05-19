package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveTrain {

    private static DcMotorEx RFDrive;
    private static DcMotorEx LFDrive;
    private static DcMotorEx RBDrive;
    private static DcMotorEx LBDrive;
    private static boolean directionFlip = false;
    private static String frontLeft  = frontLeftName;
    private static String frontRight = frontRightName;
    private static String backLeft   = backLeftName;
    private static String backRight  = backRightName;
    private static double slowdownMultiplier = 1;
    private static boolean init_ = false;
    private static HardwareMap hmap = null;
    private static DcMotorEx.ZeroPowerBehavior motor_zero_power = DcMotorEx.ZeroPowerBehavior.BRAKE;

    public static void setHardwareMap(HardwareMap map) {
        hmap = map;
    }

    public static void init(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        frontLeft = LeftFront;
        frontRight = RightFront;
        backLeft = LeftBack;
        backRight = RightBack;
        init();
    }

    public static boolean isInit() { return init_; }

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
            RFDrive = hmap.get(DcMotorEx.class, frontRight);
            LFDrive = hmap.get(DcMotorEx.class, frontLeft);
            RBDrive = hmap.get(DcMotorEx.class, backRight);
            LBDrive = hmap.get(DcMotorEx.class, backLeft);

            setMotorZeroPowerBehavior(motor_zero_power);

            LFDrive.setDirection(DcMotorEx.Direction.REVERSE);
            LBDrive.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }

    public static void update() {

        double vertical     = -ComplexGamepad.LEFT_STICK_Y1.get().raw();
        double horizontal   = -ComplexGamepad.LEFT_STICK_X1.get().raw();
        double pivot        = ComplexGamepad.RIGHT_STICK_X1.get().raw();

        if (directionFlip) {
            horizontal = - horizontal;
            vertical = - vertical;
        }

        double FrontRightPow = vertical + horizontal - pivot;
        double BackRightPow  = vertical - horizontal - pivot;
        double FrontLeftPow  = vertical - horizontal + pivot;
        double BackLeftPow   = vertical + horizontal + pivot;

        double afrp = Math.abs(FrontRightPow);
        double aflp = Math.abs(FrontLeftPow);
        double abrp = Math.abs(BackRightPow);
        double ablp = Math.abs(BackLeftPow);

        double maxp = Math.max(Math.max(Math.max(afrp, aflp), abrp), ablp);

        if (maxp > 1.0) {
            FrontRightPow /= maxp;
            FrontLeftPow /= maxp;
            BackRightPow /= maxp;
            BackLeftPow /= maxp;
        }

        RFDrive.setPower(FrontRightPow * slowdownMultiplier);
        LFDrive.setPower(FrontLeftPow * slowdownMultiplier);
        RBDrive.setPower(BackRightPow * slowdownMultiplier);
        LBDrive.setPower(BackLeftPow * slowdownMultiplier);
    }

    public static DcMotorEx.ZeroPowerBehavior getMotorZeroPowerBehavior() {
        return motor_zero_power;
    }

    public static void setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior motor_zero_power) {
        if (motor_zero_power == DcMotorEx.ZeroPowerBehavior.UNKNOWN) return;
        DriveTrain.motor_zero_power = motor_zero_power;
        RFDrive.setZeroPowerBehavior(motor_zero_power);
        LFDrive.setZeroPowerBehavior(motor_zero_power);
        RBDrive.setZeroPowerBehavior(motor_zero_power);
        LBDrive.setZeroPowerBehavior(motor_zero_power);
    }

    public void telemetry() {}
}
