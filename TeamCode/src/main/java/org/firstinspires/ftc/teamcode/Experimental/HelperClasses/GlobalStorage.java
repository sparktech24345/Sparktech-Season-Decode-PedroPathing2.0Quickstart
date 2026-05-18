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


    // CONSTANTS

    //public static Class<?> F_Constants = FConstants.class;
    //public static Class<?> L_Constants = LConstants.class;
    public static Class<?> Constants = ConstantsDecode.class;

    public static Pose startPose = new Pose(0,0,0);


    // HARDWARE NAMES
    public static String frontRightName         = "frontright";
    public static String frontLeftName          = "frontleft";
    public static String backRightName          = "backright";
    public static String backLeftName           = "backleft";
    public static ColorSet_ITD currentTeamITD = ColorSet_ITD.Undefined;
    public static double ballColorTresholdBlue = 20;
    public static double ballColorTresholdGreen = 20;
    public static double leftSensorColorMultiplier = 0.9;

    // Sensor scale: 3.3V corresponds to 1000 mm
    public static double MAX_VOLTS = 3.3;
    public static double MAX_DISTANCE_MM = 1000.0;
    public static double ballInIntakeThreshold = 235;

    // DECODE

    // DECODE ROBOT V2
    public static String intakeMotorName = "intakeMotor"; // port control 3
    public static String turretRotationMotorName = "turretRotateMotor"; // port control 2
    public static String turretFlyWheelMotorLeftName = "turretFlyWheelMotorLeft"; // port control 0
    public static String turretFlyWheelMotorRightName = "turretFlyWheelMotorRight"; // port control 1

    public static String rightGateServoName = "rightGateServo"; // port 2
    public static String leftGateServoName = "leftGateServo"; // port 3
    public static String rightTiltServoName = "rightTiltServo"; // port
    public static String leftTiltServoName = "leftTiltServo"; // port
    public static String turretAngleServoName = "turretAngleServo"; // port 4
    public static String CameraRotateServoName = "CameraRotateServo"; // port 0
    public static String PTOServoName = "PTOServo"; // port 1
    public static String colorSensorRightName = "colorSensorRight";
    public static String colorSensorLeftName = "colorSensorLeft";
    public static String distanceSensorName = "distanceSensor";

    public static OpModes currentOpModes = OpModes.TeleOP;
    public static TeamColor currentTeamColor = TeamColor.TeamNotSet;
    public static MotifSequence currentMotifSequence = MotifSequence.Undefined;
    public static BallColorSet_Decode purpleSensorBall1 = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode purpleSensorBall2 = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode greenSensorBall = BallColorSet_Decode.NoBall;
    public static BallColorSet_Decode launchSensorBall = BallColorSet_Decode.NoBall;


    //function for firing stuff
    public static Pose globalRobotPose = new Pose();
    public static Pose futureMoveActionTargetPose = new Pose();
    public static int teamPipeline = 0;
    public static double redThreshold = 40;

    public static int globalCamId = 0;

    public static double normalizeTurretRotationForServo(double targetDegrees) {
        targetDegrees = clamp(targetDegrees, -121.5, 121.5);
        return - targetDegrees + 121.5;
    }

    public static Pose pose(double x, double y, double degrees) {
        return new Pose(x, y, java.lang.Math.toRadians(degrees));
    }



    public static double farAngle = 105; // old 70
    public static double farAngleGrade0 = 177;
    public static double farAngleGrade1 = -21;
    public static double normalAngle = 160;

    public static double closeAngle = 295;
    public static double almostCloseAngle = 210;
    public static double grade1angle = -49;//-49.13547x+232.62596
    public static double grade0angle = 233;//-49.13547x+232.62596
    public static double distanceToAngleFunction(double distance) {
        int handicapAdder;
        if(distance < 1)    return closeAngle;
        if(distance < 1.7)  handicapAdder = 25;
        else handicapAdder = 0;
        if(distance > 3) return farAngle;
//        if(distance > 3) return farAngleGrade1 * distance + farAngleGrade0;

        return grade1angle * distance + grade0angle + handicapAdder;

    }
    public static double grade0VeloClose = 1060;
    public static double grade1VeloClose = 230.5;
    public static double closeVelo = 1300; //230.49196x+1048.81104
    public static double grade0farVelo = 800; // 770
    public static double grade1farVelo = 319; // 318
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
