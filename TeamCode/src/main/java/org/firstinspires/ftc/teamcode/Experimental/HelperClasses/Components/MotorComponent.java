package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

@Config
public class MotorComponent extends EncodedComponent {
    public static double voltageMultiplier = 1;
    protected HashMap<String, DcMotorEx> motorMap = new HashMap<>();
    protected DcMotorEx mainMotor = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected PIDFCoefficients CoefficientsForVPID = null;
    protected double overridePower = -2;
    protected boolean andreiOverride = false;
    protected double targetVPID = 0;
    protected boolean VPIDOverride = false;
    protected double velocity = 0;

    public MotorComponent setVoltage(double voltage) {
        this.voltage = voltage;
        return this;
    }

    protected double voltage = 12;

    public MotorComponent() {
        super();
        PID = new PIDcontroller();
    }

    public DcMotorEx getMotor(String name) {
        return motorMap.get(name);
    }

    public MotorComponent loadState(String s) {
        target = states.get(s);
        return this;
    }

    public MotorComponent useWithEncoder(boolean useWithEncoder) {
        if (useWithEncoder) componentEncoder = new Encoder(mainMotor);
        return this;
    }

    public MotorComponent setResolution(double res) {
        resolution = res;
        return this;
    }

    public MotorComponent addMotor(String hardwareMapName) {
        DcMotorEx motor = hardwareMapInstance.get(DcMotorEx.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainMotor = motor;
        }
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorMap.put(hardwareMapName, motor);
        return this;
    }
    public MotorComponent setMode(DcMotorEx.RunMode mode){
        mainMotor.setMode(mode);
        return this; // with or without encoder
    }

    public double getPosition() {
        return mainMotor.getCurrentPosition();
    }
    public double getPower() {
        return mainMotor.getPower();
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

    public MotorComponent targetVPIDOverrideBoolean(boolean VPIDOverride) {
        this.VPIDOverride = VPIDOverride;
        return this;
    }
    public MotorComponent setOverrideCondition(boolean andreiOverride) {
        this.andreiOverride = andreiOverride;
        return this;
    }
    public MotorComponent setTargetOverride(double targetVPID) {
        this.targetVPID = targetVPID;
        return this;
    }
    public MotorComponent setPowerOverride(double power) {
        this.overridePower = power;
        return this;
    }

    public MotorComponent setPIDconstants(double p, double i, double d) {
        PID.setConstants(p, i, d);
        return this;
    }
    public MotorComponent setVPIDconstants(double p, double i, double d, double f) {
        if(CoefficientsForVPID == null) CoefficientsForVPID = new PIDFCoefficients(p,i,d,f);
        for (DcMotorEx motor : motorMap.values()) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,CoefficientsForVPID);
        }
        return this;
    }

    public MotorComponent useWithPIDController(boolean b) {
        usePID = b;
        return this;
    }

    public MotorComponent setRange(double min, double max) {
        min_range = min;
        max_range = max;
        return this;
    }
    private static final double TICKS_PER_REV = 28.0;

    public DcMotorEx get(String name) {
        return motorMap.get(name);
    }

    public double getVelocity() {
        return velocity;
    }

    @Override
    public void update() {
        velocity = mainMotor.getVelocity();
        if (componentEncoder != null) {
            componentEncoder.update();
        }
        if (min_range < 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }
        double targetPower = target / resolution;
        if (usePID) {
            targetPower = PID.calculate(target, componentEncoder.getEncoderPosition());
        }
        if (andreiOverride) targetPower = overridePower;

        if (VPIDOverride)
            for (DcMotorEx motor : motorMap.values()) {
                motor.setVelocity(targetVPID);
            }
        else{
            for (DcMotorEx motor : motorMap.values()) {
                motor.setPower(targetPower);
            }
        }
    }
}
