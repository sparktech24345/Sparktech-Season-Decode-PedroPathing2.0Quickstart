package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;


public class MotorComponent extends EncodedComponent {
    protected HashMap<String, DcMotor> motorMap = new HashMap<>();
    protected DcMotor mainMotor = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected double overridePower = -2;
    protected boolean andreiOverride = false;
    protected double targetRpm = 0;
    protected boolean rpmOverride = false;

    public MotorComponent() {
        super();
        PID = new PIDcontroller();
    }

    public DcMotor getMotor(String name) {
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
        DcMotor motor = hardwareMapInstance.get(DcMotor.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainMotor = motor;
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorMap.put(hardwareMapName, motor);
        return this;
    }

    public double getPosition() {
        return mainMotor.getCurrentPosition();
    }
    public double getPower() {
        return mainMotor.getPower();
    }
    public MotorComponent setBehaviour(DcMotor.ZeroPowerBehavior zeroPower) {
        for (DcMotor motor : motorMap.values()) {
            motor.setZeroPowerBehavior(zeroPower);
        }
        return this;
    }

    public MotorComponent setDirection(String motorName, DcMotorSimple.Direction dir) {
        motorMap.get(motorName).setDirection(dir);
        return this;
    }

    public MotorComponent targetOverride(boolean rpmOverride){
        this.rpmOverride = rpmOverride;
        return this;
    }
    public MotorComponent setOverrideCondition (boolean andreiOverride){
        this.andreiOverride = andreiOverride;
        return this;
    }
    public MotorComponent setTargetOverride(double targetRpm){
        this.targetRpm = targetRpm;
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

    public MotorComponent useWithPIDController(boolean b) {
        usePID = b;
        return this;
    }

    public MotorComponent setRange(double min, double max) {
        min_range = min;
        max_range = max;
        return this;
    }

    double current_pos;
    double last_pos = 0;
    ElapsedTime pos_timer = new ElapsedTime();
    public double CalculatePower(double target_rpm){
        double error;
        double current_rpm = 0;


        if (pos_timer.milliseconds() >= 100) {
            current_pos = mainMotor.getCurrentPosition();
            double diff = current_pos - last_pos;
            diff /= 28;
            current_rpm = diff * 600;
            last_pos = current_pos;
            pos_timer.reset();
        }
        error = target_rpm - current_rpm;
        return (target_rpm + error)/3500;
    }

    public DcMotor get(String name) {
        return motorMap.get(name);
    }

    @Override
    public void update() {
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
        if(rpmOverride) targetPower = CalculatePower(targetRpm);
        for (DcMotor motor : motorMap.values()) {
            motor.setPower(targetPower);
        }
    }
}
