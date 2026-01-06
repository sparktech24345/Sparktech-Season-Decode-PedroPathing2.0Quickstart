package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.MotifSequence;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsDecode;

@Config
public class GlobalStorage {

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static double angle_clamp(double val, double min, double max) {
        while (val < min) val += Math.abs(min);
        while (val > max) val -= Math.abs(max);
        return val;
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

    // DECODE ROBOT V2
    public static String intakeMotorName = "intakeMotor"; // port control 3
    public static String turretRotationMotorName = "turretRotateMotor"; // port control 2
    public static String turretFlyWheelMotorLeftName = "turretFlyWheelMotorLeft"; // port control 0
    public static String turretFlyWheelMotorRightName = "turretFlyWheelMotorRight"; // port control 1

    public static String rightGateServoName = "rightGateServo";
    public static String leftGateServoName = "leftGateServo";
    public static String turretAngleServoName = "turretAngleServo";
    public static String intakeSorterServoName = "intakeServo";
    public static String colorSensorRightName = "colorSensorRight";
    public static String colorSensorLeftName = "colorSensorLeft";

    // OTHER S(TUFF)
    public static double slowdownConstant       = 1;
    public static OpModes currentOpModes = OpModes.TeleOP;
    public static TeamColor currentTeamColor = TeamColor.TeamNotSet;
    public static MotifSequence currentMotifSequence = MotifSequence.Undefined;
    public static BallColorSet_Decode purpleSensorBall1 = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode purpleSensorBall2 = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode greenSensorBall = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode launchSensorBall = BallColorSet_Decode.NoBall;


    //function for firing stuff

    public static double grade0 = 605.276311;
    public static double grade1 = 3.37452358;
    public static double grade2 = -0.0064362671;
    public static double grade3 = 0.0000058014683;

    public static double grade0Far = 610;
    public static double grade1Far = 325;
    public static Pose globalRobotPose = new Pose();
    public static int teamPipeline = 0;
    public static double normalizeTurretRotationForServo(double targetDegrees) {
        targetDegrees  = clamp(targetDegrees,-121.5,121.5);
        return - targetDegrees + 121.5;
    }
    public static double degreesToOuttakeTurretServo(double degrees) {
        double realDown = 76; // unghi real, masurate in cad, a nu se schimba
        double realUp = 58; // tot areal, unghi pana la blocker mecanic

        double servoDown = 136.8; // servo down * 360 actually 0.38
        double servoUp = 36; // servo up * 360 actually 0.1

        clamp(degrees,realUp, realDown); // maxing it out

        //interpolation
        return servoDown + (degrees - realDown) * (servoUp - servoDown) / (realUp - realDown);
    }
    public static double voltageMultiplierForMotor(double voltage){
        voltage = clamp(voltage,11,14);
        //set points for function
        double voltajA = 12.7;
        double multiplierA = 0.9;

        double voltajB = 12;
        double multiplierB = 1;

        return multiplierA + (voltage - voltajA) * (multiplierB - multiplierA) / (voltajB - voltajA);
    }
    public static double calculateDistance(Pose pose1, Pose pose2, boolean convertToMeters) {

        double dx = pose2.getX() - pose1.getX();
        double dy = pose2.getY() - pose1.getY();

        double distance = Math.sqrt(dx * dx + dy * dy);

        if (convertToMeters)
            // Assuming your Pose coordinates are in INCHES (FTC standard)
            distance *= 0.0254; // inches to meters

        return distance;
    }

    public static double calculateDistanceNonZero(Pose pose1, Pose pose2, boolean convertToMeters) {
        double distance = calculateDistance(pose1, pose2, convertToMeters);
        if (distance == 0) return 0.00001;
        return distance;
    }

    public static double distanceToAngleFunction(double distance) {
        if( distance > 2.9) return 270;
        if(distance < 1.05) return 310;
        else return 280; // TODO: Change this
    }
    public static double distanceToVelocityFunction(double distance) {
        if( distance > 2.9) return 1240;
        return 176.88679 * distance + 640; // TODO: Change this
    }
}
