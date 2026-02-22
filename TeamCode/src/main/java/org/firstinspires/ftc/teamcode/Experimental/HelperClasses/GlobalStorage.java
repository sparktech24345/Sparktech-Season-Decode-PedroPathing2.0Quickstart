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
    public static boolean evalForTrigger(double val) {
        return val >= 0.4;
    }
    public static boolean TriggerEval(double val) {
        return val > 0.4;
    }
    public static <Tx, Ty> Pair<Tx, Ty> make_pair(Tx arg1, Ty arg2) { return new Pair<>(arg1, arg2); }

    // INSTANCES
    public static RobotController robotController = null;


    // CONSTANTS

    //public static Class<?> F_Constants = FConstants.class;
    //public static Class<?> L_Constants = LConstants.class;
    public static Class<?> Constants = ConstantsDecode.class;

    public static Pose startPose = new Pose(0,0,0);


    // HARDWARE NAMES
    public static String frontRightName         = "frontleft";
    public static String frontLeftName          = "frontright";
    public static String backRightName          = "backleft";
    public static String backLeftName           = "backright";
    public static ColorSet_ITD currentTeamITD = ColorSet_ITD.Undefined;
    public static double ballColorTresholdBlue = 15;
    public static double ballColorTresholdGreen = 15;
    public static double leftSensorColorMultiplier = 1.5;

    // DECODE

    // DECODE ROBOT V2
    public static String intakeMotorName = "intakeMotor"; // port control 3
    public static String turretRotationMotorName = "turretRotateMotor"; // port control 2
    public static String turretFlyWheelMotorLeftName = "turretFlyWheelMotorLeft"; // port control 0
    public static String turretFlyWheelMotorRightName = "turretFlyWheelMotorRight"; // port control 1

    public static String rightGateServoName = "rightGateServo"; // port 2
    public static String leftGateServoName = "leftGateServo"; // port 3
    public static String rightTiltServoName = "rightTiltServo"; // port 1
    public static String leftTiltServoName = "leftTiltServo"; // port 0
    public static String turretAngleServoName = "turretAngleServo"; // port 4
    public static String colorSensorRightName = "colorSensorRight";
    public static String colorSensorLeftName = "colorSensorLeft";

    public static OpModes currentOpModes = OpModes.TeleOP;
    public static TeamColor currentTeamColor = TeamColor.TeamNotSet;
    public static MotifSequence currentMotifSequence = MotifSequence.Undefined;
    public static BallColorSet_Decode purpleSensorBall1 = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode purpleSensorBall2 = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode greenSensorBall = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode launchSensorBall = BallColorSet_Decode.NoBall;


    //function for firing stuff
    public static Pose globalRobotPose = new Pose();
    public static int teamPipeline = 0;
    public static double redThreshold = 40;

    public static double normalizeTurretRotationForServo(double targetDegrees) {
        targetDegrees  = clamp(targetDegrees,-121.5,121.5);
        return - targetDegrees + 121.5;
    }

    public static Pose pose(double x, double y, double degrees) {
        return new Pose(x, y, Math.toRadians(degrees));
    }

    public static double degreesToOuttakeTurretServo(double degrees) {
        double realDown = 76; // unghi real, masurate in cad, a nu se schimba
        double realUp = 58; // tot areal, unghi pana la blocker mecanic

        double servoDown = 136.8; // servo down * 360 actually 0.38
        double servoUp = 36; // servo up * 360 actually 0.1

        clamp(degrees, realUp, realDown); // maxing it out

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

        double correctedX = pose1.getX();
        if(correctedX < 0) correctedX = 0; /// if 0 is less then 0 that means you have exited field, this might be risky though

        double dx = pose2.getX() - correctedX;
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

    public static double farAngle = 105;
    public static double normalAngle = 160;

    public static double closeAngle = 310;
    public static double almostCloseAngle = 200;
    public static double grade1angle = -49;//-49.13547x+232.62596
    public static double grade0angle = 233;//-49.13547x+232.62596
    public static double distanceToAngleFunction(double distance) {
        if(distance < 1)    return closeAngle;
        if(distance < 1.25) return almostCloseAngle;
        if(distance > 3) return farAngle;
        else return grade1angle * distance + grade0angle;

    }
    public static double grade0VeloClose = 1050;
    public static double grade1VeloClose = 230.5;
    public static double closeVelo = 1300; //230.49196x+1048.81104
    public static double grade0farVelo = 760;
    public static double grade1farVelo = 318;
    // general grad \\
    public static double distanceToVelocityFunction(double  distance) {
        if (distance < 1.25) return closeVelo;
        if (distance > 3) return grade1farVelo * distance + grade0farVelo;
        return grade1VeloClose * distance + grade0VeloClose;
    }
//    public static double grade0VeloClose = 1280;
//    public static double grade1VeloClose = 100;
//    public static double grade0farVelo = 760;
//    public static double grade1farVelo = 320;
//    public static double distanceToVelocityFunction(double distance) {
//        if (distance > 2.9) return grade1farVelo * distance + grade0farVelo;;
//        return grade1VeloClose * distance + grade0VeloClose;
//    }

    public static double interpolate(double x, double x1, double x2, double y1, double y2) {
        double tangent = (y2 - y1) / (x2 - x1);
        return (y1 + tangent * (x - x1));
    }
}
