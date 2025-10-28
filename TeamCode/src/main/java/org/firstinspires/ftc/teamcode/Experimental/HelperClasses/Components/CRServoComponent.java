package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.MotionProfiler;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

public class CRServoComponent extends Component {

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
    protected AnalogInput servoEncoder;
    protected double servoAnalogPosition=-1; //an impossible value
    protected double servoAnalogTotalPosition=0;
    // i want aprox 400 degrees per sec even tho axon can push 493,333 degrees per sec, with basically instant acceleration
    protected MotionProfiler profiler = new MotionProfiler(400, 1000); // deg/sec, deg/sec^2 ( accel )
    protected ElapsedTime timer = new ElapsedTime();
    protected double lastTime = timer.seconds();

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
        if(PID == null) PID = new PIDcontroller();
        PID.setConstants(p, i, d);
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
        if(PID == null) PID = new PIDcontroller();
        usePID = b;
        return this;
    }

    public double getPosition() { return mainServo.getPower(); }
    public double getAnalogPosition(){if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * 360; //should be the position in degrees, resets under 0 or above 360
    }
    public double getServoAnalogTotalPosition(){
        return servoAnalogTotalPosition;
    }
    public void updateAnalogServoPosition(){
        double lastPosition = servoAnalogPosition;
        servoAnalogPosition = getAnalogPosition();

        double deltaPosition = servoAnalogPosition - lastPosition;
        if(lastPosition != -1) { // taking care of the first run / init as it might return something big
            if (deltaPosition > 180) {
                deltaPosition -= 360;
                //arounds--;
            } else if (deltaPosition < -180) {
                deltaPosition += 360;
                //arounds++;
            }
        }
        else deltaPosition -= (1 + 24)*(1/0.74); //taking that -1 back if it is the first run and of the 99 0

        servoAnalogTotalPosition += deltaPosition*0.74;
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
        double dt = timer.seconds() - lastTime; //timer in seconds for motion profiler cuz physics
        lastTime = timer.seconds();

        if (min_range < 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }
        if (overrideTarget_bool) {
            target = overrideTarget;
        }
        double targetPower = target / resolution;
        if (usePID) {
            double smoothTarget = profiler.update(servoAnalogTotalPosition, target, dt);  //motion profiling i think
            updateAnalogServoPosition();
            targetPower = PID.calculate(smoothTarget, servoAnalogTotalPosition);
            if(Math.abs(smoothTarget - servoAnalogTotalPosition) < 25) targetPower *= 1.5;
            if(Math.abs(smoothTarget - servoAnalogTotalPosition) < 10) targetPower *= 5;
            if(Math.abs(smoothTarget - servoAnalogTotalPosition) < 2) targetPower = 0;
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
