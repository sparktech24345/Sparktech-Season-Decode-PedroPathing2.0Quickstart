package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class ServoComponent extends Component {

    protected HashMap<String, Servo> motorMap = new HashMap<>();
    protected Servo mainServo = null;

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

    public double getPosition() {
        return mainServo.getPosition();
    }

    public Servo get(String name) {
        return motorMap.get(name);
    }

    @Override
    public void update() {
        double targetPos = target / resolution;
        if (min_range < 0 && max_range > 0) {
            targetPos = clamp(targetPos, min_range, max_range);
        }
        for (Servo motor : motorMap.values()) {
            motor.setPosition(targetPos);
        }
    }
}
