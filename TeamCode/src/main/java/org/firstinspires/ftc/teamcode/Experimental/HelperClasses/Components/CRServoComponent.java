package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.MotionProfiling.getTrapezoidPosition;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

@Config
public class CRServoComponent extends EncodedComponent {

    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected double overridePower = -2;
    protected double overrideTarget = 0;
    protected double lastPower = 0;
    protected boolean overrideTarget_bool = false;
    protected boolean overridePower_bool = false;

    // Encoder for servo position feedback
    protected AnalogInput servoEncoder = null;
    protected DcMotorEx externalEncoder = null;
    protected double servoAnalogPosition = -1; //an impossible value
    protected double externalEncoderPosition = 0;
    protected double pinpointPosition = 0; // a passer value
    protected double pinpointMathPosition = -1; //an impossible value
    protected double servoAnalogTotalPosition = 0;
    protected double pinpointTotalPosition = 0;
    protected double externalEncoderAbsolutePosition = 0;
    protected ElapsedTime timer = new ElapsedTime();
    protected double curentPos = 0;
    protected double lastCurentPos = 0;
    protected double lastLastCurentPos = 0;
    protected double lastLastLastCurentPos = 0;
    protected double maxAccel = 10000;
    protected double maxVel = 4000;
    protected double motionTime = 0.5;
    public static double encoderDirectionMulti = -1;
    protected final double encoderUnitConstant = 123.37;

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

    @Override
    public <T extends EncodedComponent> T useWithEncoder(boolean useWithEncoder) {
        return (T)this;
    }

    public double getPosition() { return mainServo.getPower(); }
    public double getAnalogPosition() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * 360; //should be the position in degrees, resets under 0 or above 360
    }
    public double getServoAnalogTotalPosition() {
        return servoAnalogTotalPosition;
    }
    public double getServoAvrgPosition() {
        return averagePosition();
    }
    public double getpinpointTotalPosition() {
        return pinpointTotalPosition;
    }
    public void setPinpointPosition(double pinpointPosition) {
        this.pinpointPosition = pinpointPosition;
    }
    public void initExternalEncoderPosition(double adder){
        externalEncoderAbsolutePosition += adder;
    }
    public void updateExternalEncoderPosition(){
        double lastPosition = externalEncoderPosition;
        externalEncoderPosition = getAnalogPosition();

        double deltaPosition = externalEncoderPosition - lastPosition;
        if (deltaPosition > 180) {
            deltaPosition -= 360;
        } else if (deltaPosition < -180) {
            deltaPosition += 360;
        }

        externalEncoderAbsolutePosition += deltaPosition;
    }
    public void updatePinpointPosition() {
        double lastPosition = pinpointMathPosition;
        pinpointMathPosition = pinpointPosition;

        double deltaPosition = pinpointMathPosition - lastPosition;
        if (lastPosition != -1) { // taking care of the first run / init as it might return something big
            if (deltaPosition > 180) {
                deltaPosition -= 360;
                //arounds--;
            } else if (deltaPosition < -180) {
                deltaPosition += 360;
                //arounds++;
            }
        }
        else {
            updateAnalogServoPosition();
            deltaPosition += -1 + getServoAnalogTotalPosition();
        }  //taking that -1 back and the initial value

        pinpointTotalPosition += deltaPosition;
    }
    public void updateAnalogServoPosition() {
        double lastPosition = servoAnalogPosition;
        servoAnalogPosition = getAnalogPosition();

        double deltaPosition = servoAnalogPosition - lastPosition;
        if (lastPosition != -1) { // taking care of the first run / init as it might return something big
            if (deltaPosition > 180) {
                deltaPosition -= 360;
                //arounds--;
            } else if (deltaPosition < -180) {
                deltaPosition += 360;
                //arounds++;
            }
        }
        else deltaPosition -= (1 + 47) * (1 / 0.74); //taking that -1 back if it is the first run and of the 99 0

        servoAnalogTotalPosition += deltaPosition * 0.74;
    }

    public double averagePosition() {
        curentPos = pinpointPosition;
        lastCurentPos = curentPos;
        lastLastCurentPos = lastCurentPos;
        lastLastLastCurentPos = lastLastCurentPos;
        return (curentPos+lastCurentPos+lastLastCurentPos+lastLastLastCurentPos)/4;
    }
    public double clampPositionTarget(double currentPos, double targetPos, double minPos, double maxPos) {
        double clampedPosition;

        double range = maxPos - minPos;
        if (range <= 0) {
            return 0; //max pos must be greater then min pos
        }

        //normalize to within one full revolution i think hopey
        double normalizedTarget = ((targetPos - minPos) % range + range) % range + minPos;

        //determine shortest path (turnaround logic) i think
        double diff = normalizedTarget - currentPos;

        //if the difference is more than half the range, wrap around the other way
        if (diff > range / 2) {
            normalizedTarget -= range;
        } else if (diff < -range / 2) {
            normalizedTarget += range;
        }

        //clamping to be sure
        clampedPosition = Math.max(minPos, Math.min(normalizedTarget, maxPos));

        return clampedPosition;
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

    public CRServoComponent setEncoder(Encoder encoder) {
        componentEncoder = encoder;
        return this;
    }
    public double getEncoderReadingFormatted() {
        if (componentEncoder == null) return 0;
        double reading = componentEncoder.getEncoderPosition() * encoderDirectionMulti / encoderUnitConstant;
        while (reading < -180) reading += 360;
        while (reading > 180) reading -= 360;
        return reading;
    }

    public double PIDturela() {
        double targetPower;
        double encoderPosition = getEncoderReadingFormatted();

            //   Fancy stuff
            //targetPower = PID.calculate(getTrapezoidPosition(target, maxVel, maxAccel, motionTime), avrg);
            //targetPower = PID.calculateSpecial(target, pinpointTotalPosition,kv,minimumPowerAdder);
            targetPower = PID.calculate(target, encoderPosition);
//            if (Math.abs(target - pinpointTotalPosition) <= 2) {
//                targetPower += minimumPowerAdder * Math.signum(targetPower);
//            }
        double error = target - encoderPosition;
        if (Math.abs(error) > 80) targetPower = 1 * Math.signum(error);  // > 80
        else if (Math.abs(error) > 50) targetPower = 0.85 * Math.signum(error); // 50 - 80
        else if (Math.abs(error) > 30) targetPower = 0.65 * Math.signum(error); // 30 - 50
        else if (Math.abs(error) > 10) targetPower = 0.12 * Math.signum(error); // 10 - 30
        else if (Math.abs(error) > 5) targetPower = 0.09 * Math.signum(error);  // 5 - 10
        else targetPower = 0; // 0 - 5

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
        if (usePID)
            targetPower = PIDturela();
        else
            targetPower = target / resolution;
        if (overridePower_bool) targetPower = overridePower;

        if (Double.isNaN(targetPower)) targetPower = 0;

        for (CRServo servo : motorMap.values()) {
            servo.setPower(targetPower);
            lastPower = targetPower;
        }
    }
}
