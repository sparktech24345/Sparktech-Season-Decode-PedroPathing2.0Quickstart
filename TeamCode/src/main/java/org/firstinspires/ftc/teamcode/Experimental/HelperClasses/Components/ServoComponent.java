package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class ServoComponent extends Component {

    public static enum ServoModes {
        Position
    }
    protected HashMap<String, Servo> motorMap = new HashMap<>();
    protected Servo mainServo = null;
    protected ServoModes servoCurrentMode = ServoModes.Position;

    public ServoComponent addMotor(String hardwareMapName) {
        Servo motor = hardwareMap.get(Servo.class, hardwareMapName);
        if (motorMap.isEmpty())
            mainServo = motor;
        motorMap.put(hardwareMapName, motor);
        return this;
    }

    public ServoComponent setOperationMode(ServoModes mode) {
        this.servoCurrentMode = mode;
        return this;
    }

    public ServoComponent setTarget(double target) {
        this.target = target;
        return this;
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
        double targetPos = target;

        if (min_range <= 0 && max_range > 0)
            targetPos = clamp(targetPos, min_range, max_range);

        targetPos /= resolution;
        for (Servo motor : motorMap.values())
            motor.setPosition(targetPos);
    }
}
