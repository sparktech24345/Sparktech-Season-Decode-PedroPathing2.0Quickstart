package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode.publicHardwareMap;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.StateSet;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class ServoComponent<S extends StateSet<ServoComponent<S>>> extends MotorizedComponent<ServoComponent<S>> {

    public final S states;
    public ServoComponent(S class_states) {
        this.states = class_states;
        this.states.init(this);
        setState(this.states.defaultState());
    }

    public ServoComponent<S> self() {
        return this;
    }

    public enum ServoModes {
        Position
    }
    protected HashMap<String, CachingServo> motorMap = new HashMap<>();
    protected CachingServo mainServo = null;
    protected ServoModes servoCurrentMode = ServoModes.Position;

    public ServoComponent<S> addMotor(String hardwareMapName) {
        CachingServo motor = new CachingServo(publicHardwareMap.get(Servo.class, hardwareMapName));
        if (motorMap.isEmpty())
            mainServo = motor;
        motorMap.put(hardwareMapName, motor);
        return self();
    }

    public ServoComponent<S> setOperationMode(ServoModes mode) {
        this.servoCurrentMode = mode;
        return self();
    }
    public double getPosition() {
        return mainServo.getPosition();
    }
    public double getPrettyPosition() {
        return mainServo.getPosition() * resolution;
    }

    public CachingServo get(String name) {
        return motorMap.get(name);
    }
    public ServoModes getOperationMode(){
        return self().servoCurrentMode;
    }

    @Override
    public void update() {
        double targetPos = target;

        targetPos /= resolution;

        if (min_range <= 0 && max_range > 0)
            targetPos = clamp(targetPos, min_range, max_range);


        for (CachingServo motor : motorMap.values())
            motor.setPosition(targetPos);
    }

    @Override
    public void telemetry() {

    }
}
