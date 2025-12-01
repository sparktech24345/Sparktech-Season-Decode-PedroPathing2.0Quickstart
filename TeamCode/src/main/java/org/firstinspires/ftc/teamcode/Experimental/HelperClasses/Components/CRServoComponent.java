package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

@Config
public class CRServoComponent extends Component {

    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected PIDcontroller PIDcamera = null;
    protected double overridePower = -2;
    protected double overrideTarget = 0;
    protected double lastPower = 0;
    protected boolean overrideTarget_bool = false;
    protected boolean overridePower_bool = false;

    // Encoder for servo position feedback
    protected AnalogInput servoEncoder = null;
    protected DcMotorEx externalEncoder = null;
    protected double externalEncoderPosition=0;
    protected double pinpointPosition = 0; // a passer value
    protected double externalEncoderAbsolutePosition=0;
    protected double curentPos = 0;
    protected double lastCurentPos = 0;
    protected double lastLastCurentPos = 0;
    protected double lastLastLastCurentPos = 0;
    protected double maxAccel = 10000;
    protected double maxVel = 4000;
    protected double motionTime = 0.5;
    public static double encoderDirectionMulti = -1;
    protected final double encoderUnitConstant = 123.37;
    protected boolean overridePIDerror = false;
    protected double errorOverride = 0;
    public static double MinErrorThreshold = 1.2;
    public static double ErrorThresholdMulti = 0;
    public static double MinVelocityThreshold = 0.13;
    public static double VelocityThresholdMulti = 4.5;

    public CRServoComponent addMotor(String hardwareMapName) {
        CRServo motor = hardwareMapInstance.get(CRServo.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainServo = motor;
        }
        motorMap.put(hardwareMapName, motor);
        return this;
    }
    public CRServoComponent addMotor(String hardwareMapName, String customMapName) {
        CRServo motor = hardwareMapInstance.get(CRServo.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainServo = motor;
        }
        motorMap.put(customMapName, motor);
        return this;
    }

    public CRServoComponent setDirection(String servoName, CRServo.Direction dir) {
        motorMap.get(servoName).setDirection(dir);
        return this;
    }
    public CRServoComponent setEncoder(String analogSensorName) {
        servoEncoder = hardwareMapInstance.get(AnalogInput.class,analogSensorName);
        return this;
    }

    public CRServoComponent setExternalEncoder(String motorName) {
        externalEncoder = hardwareMapInstance.get(DcMotorEx.class,motorName);
        return this;
    }

    public CRServoComponent setPIDconstants(double p, double i, double d) {
        if (PID == null) PID = new PIDcontroller();
        PID.setConstants(p, i, d);
        return this;
    }

    public CRServoComponent setMotionconstants(double maxVel, double maxAccel, double motionTime) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.motionTime = motionTime;
        return this;
    }
    public CRServoComponent setRange(double min, double max) {
        min_range = min;
        max_range = max;
        return this;
    }

    public CRServoComponent setPowerOverride(double power) {
        this.overridePower = power;
        return this;
    }

    public CRServoComponent setOverrideBool(boolean overrideBool) {
        this.overrideTarget_bool = overrideBool;
        return this;
    }

    public CRServoComponent setTargetOverride(double target) {
        this.overrideTarget = target;
        return this;
    }

    public CRServoComponent useWithPIDController(boolean b) {
        if (PID == null) PID = new PIDcontroller();
        usePID = b;
        return this;
    }


    public double getPosition() { return mainServo.getPower(); }
    public double getAnalogPosition() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * 360; //should be the position in degrees, resets under 0 or above 360
    }
    public void setPinpointPosition(double pinpointPosition) {
        this.pinpointPosition = pinpointPosition;
    }
    public CRServoComponent initExternalEncoderPosition(double adder) {
        externalEncoderAbsolutePosition += adder; //+ getAnalogPosition();
        return this;
    }
    // No need for turn around logic due to encoder not having a limit

//    public void updateExternalEncoderPosition(){
//        double lastPosition = externalEncoderPosition;
//        externalEncoderPosition = getEncoderReadingFormatted();
//
//        double deltaPosition = externalEncoderPosition - lastPosition;
//        if (deltaPosition > 180) {
//            deltaPosition -= 360;
//        } else if (deltaPosition < -180) {
//            deltaPosition += 360;
//        }
//
//        externalEncoderAbsolutePosition += deltaPosition;
//    }
    public double getExternalEncoderPosition(){
        return externalEncoderPosition;
    }

    public double averagePosition() {
        curentPos = pinpointPosition;
        lastCurentPos = curentPos;
        lastLastCurentPos = lastCurentPos;
        lastLastLastCurentPos = lastLastCurentPos;
        return (curentPos + lastCurentPos + lastLastCurentPos + lastLastLastCurentPos) / 4;
    }


    public double getPower() {
        return mainServo.getPower();
    }
    public double getCalculatedPower() {
        return lastPower;
    }

    public CRServo get(String name) {
        return motorMap.get(name);
    }

    public double getEncoderReadingFormatted() {
        if (externalEncoder == null) return 0;
        double reading = externalEncoder.getCurrentPosition() * encoderDirectionMulti / encoderUnitConstant;
//        while (reading < -180) reading += 360;
//        while (reading > 180) reading -= 360;
        return reading;
    }

    public static double cameraTarget = 0;
    public CRServoComponent setCameraTarget(double target) {
        cameraTarget = target;
        return this;
    }

    public double getEncoderVelocity() {
        if (externalEncoder == null) return 0;
        return externalEncoder.getVelocity();
    }

    public CRServoComponent overridePIDerror(double val, boolean condition) {
        errorOverride = val;
        overridePIDerror = condition;
        return this;
    }

    public CRServoComponent setCameraPIDconstants(double p, double i, double d) {
        if (PIDcamera == null) PIDcamera = new PIDcontroller();
        PIDcamera.setConstants(p, i, d);
        return this;
    }

    private double PID_camera() {
        if (PIDcamera == null) PIDcamera = new PIDcontroller();
        double targetPower = PIDcamera.calculate(cameraTarget, errorOverride);
        if (Math.abs(errorOverride) <= MinErrorThreshold) targetPower *= ErrorThresholdMulti;
        if (Math.abs(getEncoderVelocity()) < MinVelocityThreshold) targetPower *= VelocityThresholdMulti;
        return targetPower;
    }

    private double PID() {
        double pos = getEncoderReadingFormatted();
        double targetPower = PID.calculate(target, pos);
        if (Math.abs(target - pos) <= 1) targetPower *= 0;
        else if (Math.abs(target - curentPos) > 1 && Math.abs(getEncoderVelocity()) == 0) targetPower *= 3;
        return targetPower;
    }

    @Override
    public void update() {
        // double dt = timer.seconds() - lastTime; //timer in seconds for motion profiler cuz physics
        // lastTime = timer.seconds();

        if (min_range < 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }
        if (overrideTarget_bool) {
            target = overrideTarget;
        }
        double targetPower;
        if (usePID) {
            //updateExternalEncoderPosition();
            /*   Fancy stuff
            //targetPower = PID.calculate(getTrapezoidPosition(target, maxVel, maxAccel, motionTime), avrg);
            //targetPower = PID.calculateSpecial(target, pinpointTotalPosition,kv,minimumPowerAdder);
            targetPower = PID.calculate(target, pinpointTotalPosition);
            if (Math.abs(target - pinpointTotalPosition) <= 2) targetPower *= 0;
//            if (Math.abs(target - pinpointTotalPosition) <= 2) {
//                targetPower += minimumPowerAdder * Math.signum(targetPower);
//            }o
             */
//            double error = target - getEncoderReadingFormatted();
//            if(Math.abs(error) > 80 ) targetPower = 1 * Math.signum(error);  // >80
//            else if(Math.abs(error) > 50 ) targetPower = 0.85 * Math.signum(error); // 50 - 80
//            else if(Math.abs(error) > 30 ) targetPower = 0.65 * Math.signum(error); // 30 - 50
//            else if(Math.abs(error) > 10 ) targetPower = 0.12 * Math.signum(error); // 10 - 30
//            else if(Math.abs(error) > 5 ) targetPower = 0.09 * Math.signum(error);  // 5 - 10
//            else targetPower = 0; // 0 -5
            if (PIDcamera == null) PIDcamera = new PIDcontroller();
            if (!overridePIDerror) PIDcamera.setIntegralSum(0);
            else {
                double clamp_val = 1 / PIDcamera.getKi();
                PIDcamera.setIntegralSum(clamp(PIDcamera.getIntegralSum(), -clamp_val, clamp_val));
            }
            targetPower = (overridePIDerror ? PID_camera() : PID());
        } else {
            targetPower = target / resolution;
        }
        if (overridePower_bool) targetPower = overridePower;



        if (Double.isNaN(targetPower)) targetPower = 0;
        for (CRServo servo : motorMap.values()) {
            servo.setPower(targetPower);
            lastPower = targetPower;
        }
    }
}
