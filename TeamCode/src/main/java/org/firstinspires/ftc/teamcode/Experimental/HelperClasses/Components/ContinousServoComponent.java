package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.CRServo;

import java.util.HashMap;

public class ContinousServoComponent extends Component {

    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;

    public ContinousServoComponent addMotor(String hardwareMapName) {
        CRServo motor = hardwareMapInstance.get(CRServo.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainServo = motor;
        }
        motorMap.put(hardwareMapName, motor);
        return this;
    }
    public ContinousServoComponent addMotor(String hardwareMapName, String customMapName) {
        CRServo motor = hardwareMapInstance.get(CRServo.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainServo = motor;
        }
        motorMap.put(customMapName, motor);
        return this;
    }

    public ContinousServoComponent setDirection(String servoName, CRServo.Direction dir) {
        motorMap.get(servoName).setDirection(dir);
        return this;
    }

    public double getPos() {
        return mainServo.getPower();
    }

    public CRServo get(String name) {
        return motorMap.get(name);
    }

    @Override
    public void update() {
        double targetPos = target / resolution;
        if (range > 0) {
            targetPos = clamp(targetPos, range);
        }
        for (CRServo motor : motorMap.values()) {
            motor.setPower(targetPos);
        }
    }
}
