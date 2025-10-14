package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

public class CRServoComponent extends Component {

    protected HashMap<String, CRServo> motorMap = new HashMap<>();
    protected CRServo mainServo = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;

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

    public CRServoComponent setPIDconstants(double p, double i, double d) {
        PID.setConstants(p, i, d);
        return this;
    }

    public CRServoComponent useWithPIDController(boolean b) {
        usePID = b;
        return this;
    }

    public double getPosition() { return mainServo.getPower(); }

    public double getPower() {
        return mainServo.getPower();
    }

    public CRServo get(String name) {
        return motorMap.get(name);
    }

    @Override
    public void update() {
        double targetPos = target / resolution;
        if (min_range < 0 && max_range > 0) {
            targetPos = clamp(targetPos, min_range, max_range);
        }
        for (CRServo motor : motorMap.values()) {
            motor.setPower(targetPos);
        }
    }
}
