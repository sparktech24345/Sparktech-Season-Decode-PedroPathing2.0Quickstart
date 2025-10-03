package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
* 
*  !! IMPORTANT !!
* 
*  THE DRIVETRAIN CLASS USES THE FOLLOWING ABBREVIATIONS:
*  RF - Right Front
*  LF - Left Front
*  RB - Right Back
*  LB - Left Back
* 
* */

public class DriveTrain {

    private DcMotor RFDrive;
    private DcMotor LFDrive;
    private DcMotor RBDrive;
    private DcMotor LBDrive;
    private boolean directionFlip = false;
    private String frontLeft = frontLeftName;
    private String frontRight = frontRightName;
    private String backLeft = backLeftName;
    private String backRight = backRightName;
    private double slowdownMultiplier = 1;

    public DriveTrain() {
        init();
    }

    public DriveTrain(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        frontLeft = LeftFront;
        frontRight = RightFront;
        backLeft = LeftBack;
        backRight = RightBack;
        init();
    }

    public void setSlowdown(double slowdownMultiplier) {
        this.slowdownMultiplier = slowdownMultiplier;
    }

    public void init() {
        if (currentOpModes == OpModes.TeleOP) {
            RFDrive = hardwareMapInstance.get(DcMotor.class, frontRight);
            LFDrive = hardwareMapInstance.get(DcMotor.class, frontLeft);
            RBDrive = hardwareMapInstance.get(DcMotor.class, backRight);
            LBDrive = hardwareMapInstance.get(DcMotor.class, backLeft);

            RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void start() {}

    public void loop() {

        double vertical     =  gamepadInstance.get("LEFT_STICK_Y1").raw();  // Note: pushing stick forward gives negative value
        double horizontal   =  gamepadInstance.get("LEFT_STICK_X1").raw();
        double pivot        = -gamepadInstance.get("RIGHT_STICK_X1").raw();

        double FrontRightPow = vertical + horizontal - pivot;
        double BackRightPow  = vertical - horizontal - pivot;
        double FrontLeftPow  = vertical - horizontal + pivot;
        double BackLeftPow   = vertical + horizontal + pivot;

        RFDrive.setPower(FrontRightPow * slowdownMultiplier);
        LFDrive.setPower(FrontLeftPow * slowdownMultiplier);
        RBDrive.setPower(BackRightPow * slowdownMultiplier);
        LBDrive.setPower(BackLeftPow * slowdownMultiplier);
    }

    public void stop() {}

    public void telemetry() {
    }
}
