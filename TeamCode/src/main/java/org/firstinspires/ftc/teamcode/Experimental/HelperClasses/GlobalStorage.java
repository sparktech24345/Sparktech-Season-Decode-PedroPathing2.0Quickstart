package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import android.util.Pair;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.MotifSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;


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
    public static boolean TriggerEval(double val) {
        return val > 0.4;
    }
    public static <Tx, Ty> Pair<Tx, Ty> make_pair(Tx arg1, Ty arg2) { return new Pair<>(arg1, arg2); }

    // INSTANCES
    public static RobotController robotControllerInstance = null;


    // CONSTANTS

    //public static Class<?> F_Constants = FConstants.class;
    //public static Class<?> L_Constants = LConstants.class;
    public static Class<?> Constants = ConstantsDecode.class;

    public static Pose startPose = new Pose(0,0,0);


    // HARDWARE NAMES

    public static String intakeSpinName         = "intakespin";
    public static String intakeExtendName       = "intakemotor";
    public static String intakePosName          = "intakeRotateServo";
    public static String outtakeExtendLeftName  = "outakeleftmotor";
    public static String outtakeExtendRightName = "outakerightmotor";
    public static String outtakeArmName         = "outakeArmServo";
    public static String outtakeClawName        = "outakeSampleServo";
    public static String frontRightName         = "frontpurple";
    public static String frontLeftName          = "frontgreen";
    public static String backRightName          = "backpurple";
    public static String backLeftName           = "backgreen";
    public static String colorSensorName        = "sensorColor";
    public static ColorSet_ITD currentTeamITD = ColorSet_ITD.Undefined;

    public static String __frontLeftName        = "frontleft";
    public static String __frontRightName       = "frontright";
    public static String __backLeftName         = "backleft";
    public static String __backRightName        = "backright";

    // DECODE
    public static String LaunchMotorOneName     = "launchmotorone";
    public static String LaunchMotorTwoName     = "launchmotortwo";
    public static String ServoControlName       = "controlservo";
    public static String ServoExpansionName     = "expansionservo";
    public static String PushServoOneName       = "expansionpushservo";
    public static String PushServoTwoName       = "controlpushservo";

    // OTHER S(TUFF)
    public static double slowdownConstant       = 1;
    public static OpModes currentOpModes = OpModes.TeleOP;
    public static TeamColor currentTeamColor = TeamColor.TeamNotSet;
    public static MotifSequence currentMotifSequence = MotifSequence.Undefined;
    public static BallColorSet_Decode purpleSensorBall = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode greenSensorBall = BallColorSet_Decode.NoBall;


    //function for firing stuff

    public static double grade0 = 665.276311;
    public static double grade1 = 3.37452358;
    public static double grade2 = -0.0064362671;
    public static double grade3 = 0.0000058014683;
}
