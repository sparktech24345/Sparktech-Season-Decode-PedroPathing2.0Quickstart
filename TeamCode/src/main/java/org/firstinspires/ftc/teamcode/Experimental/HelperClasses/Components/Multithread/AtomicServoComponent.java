package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Multithread;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.hardwareMapInstance;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class AtomicServoComponent extends AtomicComponent {

    protected AtomicReference<HashMap<String, Servo>> motorMap = new AtomicReference<>(new HashMap<>());
    protected AtomicReference<Servo> mainServo = null;
    protected AtomicBoolean overrideTarget_bool = new AtomicBoolean(false);
    protected AtomicReference<Double> overrideTargetPos = new AtomicReference<>(0.0);

    public AtomicServoComponent addMotor(String hardwareMapName) {
        Servo motor = hardwareMapInstance.get(Servo.class, hardwareMapName);
        if (motorMap.get().isEmpty()) {
            mainServo.set(motor);
        }
        motorMap.get().put(hardwareMapName, motor);
        return this;
    }

    public AtomicServoComponent addMotor(String hardwareMapName, String customMapName) {
        Servo motor = hardwareMapInstance.get(Servo.class, hardwareMapName);
        if (motorMap.get().isEmpty()) {
            mainServo.set(motor);
        }
        motorMap.get().put(customMapName, motor);
        return this;
    }
    public AtomicServoComponent setOverrideTarget_bool(boolean shouldOverride) {
        this.overrideTarget_bool.set(shouldOverride);
        return this;
    }
    public void setOverrideTargetPos(double overrideTargetPos) {
        this.overrideTargetPos.set(overrideTargetPos);
    }

    public double getPosition() {
        return mainServo.get().getPosition();
    }

    public Servo get(String name) {
        return motorMap.get().get(name);
    }

    @Override
    public void update() {
        double targetPos = target.get() / resolution.get();
        if (min_range.get() < 0 && max_range.get() > 0) {
            targetPos = clamp(targetPos, min_range.get(), max_range.get());
        }

        if(overrideTarget_bool.get()) targetPos = overrideTargetPos.get() / resolution.get();

        for (Servo motor : motorMap.get().values()) {
            motor.setPosition(targetPos);
        }
    }
}
