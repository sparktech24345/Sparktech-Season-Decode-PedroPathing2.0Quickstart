package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

public class CRServoComponent extends Component {

    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected double overridePower = -2;

    // Encoder for servo position feedback
    protected AnalogInput servoEncoder;
    protected double servoAnalogPosition=-1; //an impossible value
    protected double servoAnalogTotalPosition=0;

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
        else deltaPosition--; //taking that -1 back if it is the first run

        servoAnalogTotalPosition += deltaPosition;
    }


    public double getPower() {
        return mainServo.getPower();
    }

    public CRServo get(String name) {
        return motorMap.get(name);
    }

    @Override
    public void update() {
        if (min_range < 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }
        double targetPower = target / resolution;
        if (usePID) {
            updateAnalogServoPosition();
            targetPower = PID.calculate(target, servoAnalogTotalPosition);
        }
        if (overridePower > -2) targetPower = overridePower;
        for (CRServo servo : motorMap.values()) {
            servo.setPower(targetPower);
        }
    }
}
