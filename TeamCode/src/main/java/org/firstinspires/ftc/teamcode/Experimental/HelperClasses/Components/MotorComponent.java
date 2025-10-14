package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;


public class MotorComponent extends EncodedComponent {
    protected HashMap<String, DcMotor> motorMap = new HashMap<>();
    protected DcMotor mainMotor = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected double overridePower = -1;

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

    public DcMotor get(String name) {
        return motorMap.get(name);
    }

    @Override
    public void update() {
        componentEncoder.update();
        if (min_range < 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }
        double targetPower = target / resolution;
        if (usePID) {
            targetPower = PID.calculate(target, componentEncoder.getEncoderPosition());
        }
        if (overridePower > -1) targetPower = overridePower;
        for (DcMotor motor : motorMap.values()) {
            motor.setPower(targetPower);
        }
    }
}
