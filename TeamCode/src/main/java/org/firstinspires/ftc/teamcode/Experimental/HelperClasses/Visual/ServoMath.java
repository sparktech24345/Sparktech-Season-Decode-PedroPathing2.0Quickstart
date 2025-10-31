package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Visual;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

public class ServoMath {

    private RobotController robot;
    private GoBildaPinpointDriver pinpointTurret;
    protected AnalogInput servoEncoder;
    protected CRServo mainServo = null;
    protected double servoAnalogPosition=-1; //an impossible value
    protected double pinpointPosition = 0; // a passer value
    protected double pinpointMathPosition = -1; //an impossible value
    protected double servoAnalogTotalPosition=0;
    protected double pinpointTotalPosition=0;
    protected double curentPos=0;
    protected double lastCurentPos=0;
    protected double lastLastCurentPos=0;
    protected double lastLastLastCurentPos=0;


//1
    public double calculateHeadingAdjustment(Pose robotPose, double targetX, double targetY) {
        // Current robot position
        double x = robotPose.getX();
        double y = robotPose.getY(); // don't invert Y unless your coordinate system specifically requires it

        // Vector from robot to target
        double dx = targetX - x;
        double dy = targetY - y;

        // Angle from robot to target (in radians → degrees)
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Normalize to [0, 360)
        if (targetAngleDeg < 0) targetAngleDeg += 360;

        // Robot heading in degrees
        // Assuming robotPose.getHeading() is in radians, 0° = facing +X, increases counterclockwise
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Normalize to [0, 360)
        if (robotHeadingDeg < 0) robotHeadingDeg += 360;

        // Calculate smallest signed angle difference: [-180, 180]
        double angleDiff = targetAngleDeg - robotHeadingDeg;
        angleDiff = ((angleDiff + 540) % 360) - 180;  // neat trick for wrapping to [-180, 180]


        // Positive = target to the robot's right (clockwise turn), negative = to the left
        robot.addTelemetryData("CALCUL Robot heading", robotHeadingDeg);
        robot.addTelemetryData("CALCUL Target angle", targetAngleDeg);
        robot.addTelemetryData("CALCUL Heading adjustment", angleDiff);

        return angleDiff;
    }

    //2
    public double getPosition() { return mainServo.getPower(); }
    public double getAnalogPosition(){if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * 360; //should be the position in degrees, resets under 0 or above 360
    }
    public double getServoAnalogTotalPosition(){
        return servoAnalogTotalPosition;
    }
    public double getServoAvrgPosition(){
        return averagePosition();
    }
    public double getpinpointTotalPosition(){
        return pinpointTotalPosition;
    }
    public void setPinpointPosition(double pinpointPosition){
        this.pinpointPosition = pinpointPosition;
    }

    public void updateAnalogServoPosition(){
        double lastPosition = servoAnalogPosition;
        servoAnalogPosition = getAnalogPosition();

        double deltaPosition = servoAnalogPosition - lastPosition;
        if(lastPosition != -1) {
            if (deltaPosition > 180) {
                deltaPosition -= 360;
            } else if (deltaPosition < -180) {
                deltaPosition += 360;
            }
        }
        else deltaPosition -= (1 + 24 + 189)*(1/0.74);

        servoAnalogTotalPosition += deltaPosition*0.74;
    }

    public double averagePosition(){
        curentPos = pinpointPosition;
        lastCurentPos = curentPos;
        lastLastCurentPos = lastCurentPos;
        lastLastLastCurentPos = lastLastCurentPos;
        return (curentPos+lastCurentPos+lastLastCurentPos+lastLastLastCurentPos)/4;
    }


    //3
    public void getPinpointTurretPosition(){
        pinpointTurret.update();
        if(pinpointTurret != null){
            double diffAngleFromDriveTrain = pinpointTurret.getHeading(AngleUnit.DEGREES) - Math.toDegrees(robot.getCurrentPose().getHeading());
            robot.getCRServoComponent("TurretRotate").setPinpointPosition(diffAngleFromDriveTrain);
        }
        else robot.getCRServoComponent("TurretRotate").setPinpointPosition(0);
    }
    public void updatePinpointPosition(){
        double lastPosition = pinpointMathPosition;
        pinpointMathPosition = pinpointPosition;

        double deltaPosition = pinpointMathPosition - lastPosition;
        if(lastPosition != -1) {
            if (deltaPosition > 180) {
                deltaPosition -= 360;
            } else if (deltaPosition < -180) {
                deltaPosition += 360;
            }
        }
        else{
            updateAnalogServoPosition();
            deltaPosition += -1 + getServoAnalogTotalPosition();
        }

        pinpointTotalPosition += deltaPosition;
    }



}
