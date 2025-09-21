package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import android.util.Pair;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class GlobalStorage {

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static double eval(boolean val) {
        return (val ? 1 : 0);
    }
    public static boolean eval(double val) {
        return val != 0;
    }
    public static <Tx, Ty> Pair<Tx, Ty> make_pair(Tx arg1, Ty arg2) { return new Pair<>(arg1, arg2); }

    // INSTANCES

    public static ComplexFollower followerInstance = null;
    public static ComplexGamepad gamepadInstance = null;
    public static HardwareMap hardwareMapInstance = null;
    public static Telemetry telemetryInstance = null;
    public static StateQueuer queuerInstance = null;
    public static RobotController robotControllerInstance = null;


    // CONSTANTS

    //public static Class<?> F_Constants = FConstants.class;
    //public static Class<?> L_Constants = LConstants.class;
    public static Class<?> Constants = Constants.class;

    public static Pose startPose = new Pose(0,0,0);


    // HARDWARE NAMES

    public static String intakeSpinName         = "intakespin";
    public static String intakeExtendName       = "intakemotor";
    public static String intakePosName          = "intakeRotateServo";
    public static String outtakeExtendLeftName  = "outakeleftmotor";
    public static String outtakeExtendRightName = "outakerightmotor";
    public static String outtakeArmName         = "outakeArmServo";
    public static String outtakeClawName        = "outakeSampleServo";
    public static String frontRightName         = "frontright";
    public static String frontLeftName          = "frontleft";
    public static String backRightName          = "backright";
    public static String backLeftName           = "backleft";
    public static String colorSensorName        = "sensorColor";

    // DECODE

    public static String LaunchMotorOneName         = "launchmotorone";
    public static String LaunchMotorTwoName        = "launchmotortwo";
    public static String TransferServoControlHubName        = "controlservo";
    public static String TransferServoExpansionHubName        = "expansionservo";
    public static String PushServoOneName        = "expansionpushservo";
    public static String PushServoTwoName        = "controlpushservo";
    public static double slowdownConstant = 1;

    // OTHER S(TUFF)
    public static ColorSet currentTeam = ColorSet.Undefined;
    public static OpModes currentOpModes = OpModes.TeleOP;
}
