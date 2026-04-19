package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode.publicHardwareMap;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

@Config
public class CRServoComponent<S extends StateSet<CRServoComponent<S>>> extends MotorizedComponent<CRServoComponent<S>> {

    public final S states;
    public CRServoComponent(S class_states) {
        this.states = class_states;
        this.states.own(this);
        setState(this.states.defaultState());
    }

    public enum CRServoModes {
        Power
    }
    protected HashMap<String, CachingCRServo> motorMap = new HashMap<>();
    protected CachingCRServo mainServo = null;
    protected CRServoModes CRServoCurrentMode = CRServoModes.Power;
    
    public CRServoComponent addMotor(String hardwareMapName) {
        CachingCRServo motor = new CachingCRServo(publicHardwareMap.get(CRServo.class, hardwareMapName));
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

    public double getPosition() {
        return mainServo.getPower();
    }

    public double getPower() {
        return mainServo.getPower();
    }

    public CachingCRServo get(String name) {
        return motorMap.get(name);
    }
    public CRServoModes getOperationMode() {
        return this.CRServoCurrentMode;
    }

    @Override
    public void update() {
        if (min_range <= 0 && max_range > 0)
            target = clamp(target, min_range, max_range);
        for (CachingCRServo servo : motorMap.values())
            servo.setPower(target);
    }

    @Override
    public void telemetry() {

    }
}
