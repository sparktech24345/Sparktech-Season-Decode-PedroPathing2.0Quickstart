package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.HashMap;

@Config
public class CRServoComponent extends Component {

    public static enum CRServoModes{
        executeState,
        powerOverride
    }
    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;
    protected double powerOverrideValue = -2;
    protected double overrideTarget = 0;
    protected double lastPower = 0;
    protected CRServoModes CRServoCurrentMode = CRServoModes.executeState;

    public CRServoComponent(){
        super();
        this.CRServoCurrentMode = CRServoModes.executeState;
    }
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
    public CRServoComponent setPowerOverride(double power) {
        this.powerOverrideValue = power;
        return this;
    }
    public CRServoComponent setOperationMode(CRServoModes mode){
        this.CRServoCurrentMode = mode;
        return this;
    }

    public double getPosition() { return mainServo.getPower(); }

    public double getPower() {
        return mainServo.getPower();
    }

    public CRServo get(String name) {
        return motorMap.get(name);
    }
    public CRServoModes getOperationMode(){
        return this.CRServoCurrentMode;
    }

    @Override
    public void update() {
        if (min_range < 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }

        switch (CRServoCurrentMode){
            case executeState:
                for (CRServo servo : motorMap.values()) {
                    servo.setPower(target);
                    lastPower = target;
                }
            case powerOverride:
                target = powerOverrideValue;
                for (CRServo servo : motorMap.values()) {
                    servo.setPower(target);
                    lastPower = target;
                }
        }
    }
}
