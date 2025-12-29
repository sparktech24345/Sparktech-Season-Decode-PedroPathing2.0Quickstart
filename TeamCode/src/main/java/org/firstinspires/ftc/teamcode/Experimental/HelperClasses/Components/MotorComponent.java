package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.HashMap;

@Config
public class MotorComponent extends Component {
    public static enum MotorModes {
        Power,
        Velocity,
        Position
    }
    protected HashMap<String, DcMotorEx> motorMap = new HashMap<>();
    protected DcMotorEx mainMotor = null;
    protected PIDFCoefficients VelocityCoefficients = null;
    protected PIDFCoefficients PositionCoefficients = null;
    protected double positionTolerance = 0;
    protected MotorModes motorCurrentMode = MotorModes.Power;
    protected double velocity = 0;
    protected boolean isOverriden = false;
    
    public MotorComponent addMotor(String hardwareMapName) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainMotor = motor;
        }
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorMap.put(hardwareMapName, motor);
        return this;
    }

    public MotorComponent loadState(String s) {
        target = states.get(s);
        isOverriden = false;
        return this;
    }

    public MotorComponent setResolution(double res) {
        resolution = res;
        return this;
    }

    public MotorComponent setRange(double min, double max) {
        min_range = min;
        max_range = max;
        return this;
    }

    public MotorComponent setDcMotorMode(DcMotorEx.RunMode mode) {
        mainMotor.setMode(mode);
        return this; // with or without encoder
    }
    public MotorComponent setBehaviour(DcMotorEx.ZeroPowerBehavior zeroPower) {
        for (DcMotorEx motor : motorMap.values()) {
            motor.setZeroPowerBehavior(zeroPower);
        }
        return this;
    }

    public MotorComponent setDirection(String motorName, DcMotorEx.Direction dir) {
        motorMap.get(motorName).setDirection(dir);
        return this;
    }
    public MotorComponent setOperationMode(MotorModes mode) {
        if (motorCurrentMode == mode) return this;
        switch (mode) {
            case Position:
                if (PositionCoefficients != null)
                    PositionCoefficients = new PIDFCoefficients(0, 0, 0, 0);
                for (DcMotorEx motor : motorMap.values()) {
                    motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PositionCoefficients);
                    motor.setTargetPositionTolerance((int)positionTolerance);
                }
                break;
            case Velocity:
                if (VelocityCoefficients != null)
                    VelocityCoefficients = new PIDFCoefficients(0, 0, 0, 0);
                for (DcMotorEx motor : motorMap.values())
                    motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, VelocityCoefficients);
                break;
        }
        motorCurrentMode = mode;
        return this;
    }

    // ================== Set target stuff ==================
    public MotorComponent setTarget(double target) {
        this.target = target;
        isOverriden = true;
        return this;
    }

    // ================== Set PID Coefficients ==================

    public MotorComponent setPositionCoefficients(double p, double i, double d, double f, double positionTolerance) {
        PositionCoefficients = new PIDFCoefficients(p, i, d, f);
        for (DcMotorEx motor : motorMap.values()) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PositionCoefficients);
            motor.setTargetPositionTolerance((int) positionTolerance);
        }
        this.positionTolerance = positionTolerance;
        return this;
    }
    public MotorComponent setVelocityCoefficients(double p, double i, double d, double f) {
        VelocityCoefficients = new PIDFCoefficients(p, i, d, f);
        for (DcMotorEx motor : motorMap.values())
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, VelocityCoefficients);
        return this;
    }

    // ================== Getters ==================

    public boolean isOverriden() {
        return isOverriden;
    }
    public double getVelocity() {
        return velocity;
    }
    public MotorModes getOperationMode() {
        return this.motorCurrentMode;
    }
    public double getPosition() {
        return mainMotor.getCurrentPosition();
    }
    public double getPower() {
        return mainMotor.getPower();
    }
    public DcMotorEx getMotor(String name) {
        return motorMap.get(name);
    }


    // ================== Main Loop ==================

    @Override
    public void update() {
        velocity = mainMotor.getVelocity();

        if (min_range <= 0 && max_range > 0)
            target = clamp(target, min_range, max_range);
        double targetPower = target / resolution;

        switch (motorCurrentMode) {
            // ================== state stuff ==================
            case Power:
                for (DcMotorEx motor : motorMap.values())
                    motor.setPower(targetPower);
                break;

            case Position:
                mainMotor.setTargetPosition((int)target);
                targetPower = mainMotor.getPower();
                for (DcMotorEx motor : motorMap.values())
                    motor.setPower(targetPower);
                break;

            case Velocity:
                mainMotor.setVelocity(target); // this is so that we can have only 1 encoder per system of 1 or more engines on the same shaft
                targetPower = mainMotor.getPower();
                for (DcMotorEx motor : motorMap.values())
                    motor.setPower(targetPower);
                break;
        }
    }
}
