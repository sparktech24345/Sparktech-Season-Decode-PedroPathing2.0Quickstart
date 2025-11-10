package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.MotionProfiling.getTrapezoidPosition;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

@Config
public class CRServoComponent extends Component {

    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected double overridePower = -2;
    public static double kf =0;
    protected double overrideTarget = 0;
    protected double lastPower = 0;
    protected boolean overrideTarget_bool = false;
    protected boolean overridePower_bool = false;

    // Encoder for servo position feedback
    protected AnalogInput servoEncoder;
    protected double servoAnalogPosition=-1; //an impossible value
    protected double pinpointPosition = 0; // a passer value
    protected double pinpointMathPosition = -1; //an impossible value
    protected double servoAnalogTotalPosition=0;
    protected double pinpointTotalPosition=0;
    protected ElapsedTime timer = new ElapsedTime();
    protected double lastTime = timer.seconds();
    protected double curentPos=0;
    protected double lastCurentPos=0;
    protected double lastLastCurentPos=0;
    protected double lastLastLastCurentPos=0;
    protected long lastMonitorTime =0;
    protected double maxAccel=10000;
    protected double maxVel=4000;
    protected double motionTime=0.5;

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
        else deltaPosition -= (1 + 24 + 189) * (1 / 0.74); //taking that -1 back if it is the first run and of the 99 0

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
        double targetPower = target / resolution;
        if (usePID) {
            updateAnalogServoPosition();
            updatePinpointPosition();
            double avrg = pinpointTotalPosition;
            //double clampedTarget = clampPositionTarget(avrg,target,-200,200);
            targetPower = PID.calculate(getTrapezoidPosition(target, maxVel, maxAccel, motionTime), avrg);
            double feedForward = Math.cos(Math.toRadians(target)) * kf;
            targetPower += feedForward; // for now leave 0 but could pay with it
            //if (Math.abs(target - avrg) < 25) targetPower *= 1.3;
            //if (Math.abs(target - avrg) < 13) targetPower *= 1.1;
             if (Math.abs(target - avrg) <= 1) targetPower *= 0.2;
        } else {
            targetPower = target;
        }
        if (overridePower_bool) targetPower = overridePower;
        for (CRServo servo : motorMap.values()) {
            servo.setPower(targetPower);
            lastPower = targetPower;
        }
    }
}
