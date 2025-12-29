package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.HashMap;

@Config
public class CRServoComponent extends Component {

    public static enum CRServoModes {
        Power // just this for now since PID controllers dont work well on CRServos
    }
    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;
    protected CRServoModes CRServoCurrentMode = CRServoModes.Power;
    
    public CRServoComponent addMotor(String hardwareMapName) {
        CRServo motor = hardwareMap.get(CRServo.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainServo = motor;
        }
        motorMap.put(hardwareMapName, motor);
        return this;
    }

    public CRServoComponent setDirection(String servoName, CRServo.Direction dir) {
        motorMap.get(servoName).setDirection(dir);
        return this;
    }
    
    public CRServoComponent setOperationMode(CRServoModes mode) {
        this.CRServoCurrentMode = mode;
        return this;
    }

    public CRServoComponent setTarget(double target) {
        this.target = target;
        return this;
    }

    public double getPosition() {
        return mainServo.getPower();
    }

    public double getPower() {
        return mainServo.getPower();
    }

    public CRServo get(String name) {
        return motorMap.get(name);
    }
    public CRServoModes getOperationMode() {
        return this.CRServoCurrentMode;
    }

    @Override
    public void update() {
        if (min_range <= 0 && max_range > 0)
            target = clamp(target, min_range, max_range);
        for (CRServo servo : motorMap.values())
            servo.setPower(target);
    }
}
