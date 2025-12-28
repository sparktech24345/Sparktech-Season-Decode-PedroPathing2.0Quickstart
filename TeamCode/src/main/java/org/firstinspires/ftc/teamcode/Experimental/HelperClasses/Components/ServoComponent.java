package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class ServoComponent extends Component {

    public static enum ServoModes{
        executeState,
        positionOverride
    }
    protected HashMap<String, Servo> motorMap = new HashMap<>();
    protected Servo mainServo = null;
    protected double overrideTargetPos = 0;
    protected ServoModes servoCurrentMode = ServoModes.executeState;

    public ServoComponent(){
        super();
        servoCurrentMode = ServoModes.executeState;
    }

    public ServoComponent addMotor(String hardwareMapName) {
        Servo motor = hardwareMapInstance.get(Servo.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainServo = motor;
        }
        motorMap.put(hardwareMapName, motor);
        return this;
    }

    public ServoComponent addMotor(String hardwareMapName, String customMapName) {
        Servo motor = hardwareMapInstance.get(Servo.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainServo = motor;
        }
        motorMap.put(customMapName, motor);
        return this;
    }
    public ServoComponent setOperationMode(ServoModes mode){
        this.servoCurrentMode = mode;
        return this;
    }

    public void setOverrideTargetPos(double overrideTargetPos) {
        this.overrideTargetPos = overrideTargetPos;
    }

    public double getPosition() {
        return mainServo.getPosition();
    }
    public double getPrettyPosition() {
        return mainServo.getPosition() * resolution;
    }

    public Servo get(String name) {
        return motorMap.get(name);
    }
    public ServoModes getOperationMode(){
        return this.servoCurrentMode;
    }

    @Override
    public void update() {
        double targetPos = target / resolution;
        if (min_range < 0 && max_range > 0) {
            targetPos = clamp(targetPos, min_range, max_range);
        }

        switch (servoCurrentMode){
            case executeState:
                for (Servo motor : motorMap.values()) {
                    motor.setPosition(targetPos);
                }
                break;

            case positionOverride:
                targetPos = overrideTargetPos / resolution;
                for (Servo motor : motorMap.values()) {
                    motor.setPosition(targetPos);
                }
                break;
        }


    }
}
