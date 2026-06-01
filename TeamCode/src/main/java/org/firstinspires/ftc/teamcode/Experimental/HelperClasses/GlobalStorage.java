package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    public static String frontRightName         = "frontright";
    public static String frontLeftName          = "frontleft";
    public static String backRightName          = "backright";
    public static String backLeftName           = "backleft";
    public static ColorSet_ITD currentTeamITD = ColorSet_ITD.Undefined;
    public static double ballColorTresholdBlue = 6;
    public static double ballColorTresholdGreen = 6;
    public static double leftSensorColorMultiplier = 0.6;

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
    public static String rightTiltServoName = "rightTiltServo"; // port 0 expansion
    public static String leftTiltServoName = "leftTiltServo"; // port 1 expansion
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
    public static Pose CalculatedByCameraBotPose = new Pose();
    public static int teamPipeline = 0;
    public static double redThreshold = 40;

    public static int globalCamId = 0;

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

    // hard vibe coding from here on down because math is hard

    // ==========================================
    // ANGLE STUFF & FUNCTION
    // ==========================================
    public static double closeAngle = 320;       // Max lob at 0.8m
    public static double farAngle = 100;
    public static double grade1angle = -205.0;
    public static double grade0angle = 485.0;

    public static double distanceToAngleFunction(double distance) {
        if (distance <= 0.8) return closeAngle;
        if (distance >= 2.0) return farAngle; // Bottomed out at 100
        return grade1angle * distance + grade0angle;
    }
    public static double airSortingFunctionAngle(double distance) {
        return closeAngle;
    }

    // ==========================================
    // VELOCITY STUFF & FUNCTION
    // ==========================================
    public static double closeVelo = 1140;
    public static double grade1farVelo = 290;
    public static double grade0farVelo = 570;
    public static double grade1VeloClose = 200;
    public static double grade0VeloClose = 840;
    public static double airSortBias = 70;

    public static double distanceToVelocityFunction(double distance) {
        if (distance <= 1.0) return closeVelo;
        if (distance > 2.9)  return grade1farVelo * distance + grade0farVelo;

        return grade1VeloClose * distance + grade0VeloClose;
    }

    public static double airSortingFunctionVelocity(double distance) {
        if (distance <= 1.0) return closeVelo + airSortBias;
        if (distance > 2.9)  return grade1farVelo * distance + grade0farVelo + airSortBias;

        return grade1VeloClose * distance + grade0VeloClose + airSortBias;
    }

    // ==========================================
    // UTILITY
    // ==========================================
    public static double interpolate(double x, double x1, double x2, double y1, double y2) {
        double tangent = (y2 - y1) / (x2 - x1);
        return (y1 + tangent * (x - x1));
    }

    public static double camOffsetX = 6.2;
    //
    public static double calculateCameraAngle(
            double targetX, double targetY,
            Pose robotpose,
            double camOffsetX, double camOffsetY) {

        double robotX = robotpose.getX();
        double robotY = robotpose.getY();
        double robotAngle = Math.toDegrees(robotpose.getHeading());

        // Pasul 1: Transformăm unghiul robotului în radiani pentru funcțiile matematice
        double robotRad = Math.toRadians(robotAngle);

        // Pasul 2: Rotim offset-ul camerei în funcție de unghiul actual al robotului
        // (Folosim matricea de rotație standard pentru a afla unde se află camera în teren)
        double rotatedCamX = camOffsetX * Math.cos(robotRad) - camOffsetY * Math.sin(robotRad);
        double rotatedCamY = camOffsetX * Math.sin(robotRad) + camOffsetY * Math.cos(robotRad);

        // Poziția absolută a camerei în sistemul de coordonate al terenului
        double absoluteCamX = robotX + rotatedCamX;
        double absoluteCamY = robotY + rotatedCamY;

        // Pasul 3: Calculăm diferența pe X și Y de la cameră la țintă
        double deltaX = targetX - absoluteCamX;
        double deltaY = targetY - absoluteCamY;

        // Pasul 4: Calculăm unghiul absolut (în teren) către țintă folosind Math.atan2
        double absoluteTargetAngleRad = Math.atan2(deltaY, deltaX);
        double absoluteTargetAngleDeg = Math.toDegrees(absoluteTargetAngleRad);

        // Pasul 5: Unghiul camerei trebuie să fie relativ la corpul robotului
        double relativeCamAngle = absoluteTargetAngleDeg - robotAngle;

        // Pasul 6: Normalizăm unghiul în intervalul [-180, 180] grade
        while (relativeCamAngle > 180) relativeCamAngle -= 360;
        while (relativeCamAngle <= -180) relativeCamAngle += 360;

        return relativeCamAngle;
    }
    public static double convertCamAngleToServoValue(double relativeAngle) {
        double servoValue = 0.49 + (relativeAngle * (0.29 / 90.0));

        if (servoValue < 0.0) servoValue = 0.0;
        if (servoValue > 1.0) servoValue = 1.0;

        return servoValue * 360;
    }

    public static Pose relocalizeRobot(
            double camX, double camY, double camDegrees,
            double camOffsetX, double camOffsetY, double servoAngle) {

        // --- PASUL 1: Calculăm unghiul absolut al robotului ---
        // Mai întâi, convertim unghiul inversat al camerei în unghiul absolut al capului camerei (în sistemul robotului)
        double absoluteCamAngle = 180.0 - camDegrees;

        // Deoarece servoAngle este unghiul camerei FAȚĂ de robot (Camera = Robot + Servo),
        // înseamnă că unghiul absolut al robotului este: Robot = Camera - Servo
        double robotAngle = absoluteCamAngle - servoAngle;

        // Normalizăm unghiul robotului între [-180, 180]
        while (robotAngle > 180)  robotAngle -= 360;
        while (robotAngle <= -180) robotAngle += 360;

        // --- PASUL 2: Rotim offset-ul fizic al camerei în coordonate globale ---
        double robotRad = Math.toRadians(robotAngle);

        // Calculăm unde se află poziția relativă a camerei față de centrul robotului la acest unghi
        double rotatedOffsetX = camOffsetX * Math.cos(robotRad) - camOffsetY * Math.sin(robotRad);
        double rotatedOffsetY = camOffsetX * Math.sin(robotRad) + camOffsetY * Math.cos(robotRad);

        // --- PASUL 3: Aflăm poziția robotului ---
        // Robotul se află la poziția camerei MINUS acest offset rotit
        double robotX = camX - rotatedOffsetX;
        double robotY = camY - rotatedOffsetY;

        return pose(robotX, robotY, robotAngle); // pose = return new Pose(x, y, Math.toRadians(degrees)); fyi
    }
}
