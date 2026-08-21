package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

@Config
public class MotorComponent extends Component {

    public enum MotorModes {
        Power,
        AcceleratingVelocity,
        Position
    }

    protected HashMap<String, DcMotorEx> motorMap = new HashMap<>();
    protected DcMotorEx mainMotor = null;

    protected PIDcontroller pidControllerForPosition;
    protected PIDcontroller VPIDController;
    protected double vpidF = 0;
    protected double vpidS = 0;

    protected MotorModes motorCurrentMode = MotorModes.Power;
    protected double zeroVelocityMultiplier = 0;
    protected boolean isOverriden = false;
    protected boolean voltageCompensation = false;
    protected double targetVoltage = 12;

    protected double readVelocity = 0;
    protected double readPosition = 0;
    protected boolean shouldReadAmps = false;
    protected boolean shouldReadPositionAndVelo = false;
    protected double readAmps = 0;

    protected double power = 0;
    protected double lastPower = -999.0; // Initialized out of range to force first write

    // Velocity software tracking
    private final ElapsedTime velocityTimer = new ElapsedTime();
    private double lastPositionForVel = 0;

    public MotorComponent addMotor(String hardwareMapName) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainMotor = motor;
        }
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

    public MotorComponent setVoltageCompensation(boolean val) {
        this.voltageCompensation = val;
        return this;
    }

    public MotorComponent setShouldReadVoltage(boolean val) {
        this.shouldReadAmps = val;
        return this;
    }
    public MotorComponent setShouldReadPositionAndVelo(boolean val) {
        this.shouldReadPositionAndVelo = val;
        return this;
    }

    public MotorComponent setTargetVoltage(double targetVoltage) {
        this.targetVoltage = targetVoltage;
        return this;
    }

    public MotorComponent setRange(double min, double max) {
        min_range = min;
        max_range = max;
        return this;
    }

    public MotorComponent setDcMotorMode(DcMotorEx.RunMode mode) {
        mainMotor.setMode(mode);
        return this;
    }

    public MotorComponent setBehaviour(DcMotorEx.ZeroPowerBehavior zeroPower) {
        for (DcMotorEx motor : motorMap.values()) {
            motor.setZeroPowerBehavior(zeroPower);
        }
        return this;
    }

    public MotorComponent setDirection(String motorName, DcMotorEx.Direction dir) {
        DcMotorEx motor = motorMap.get(motorName);
        if (motor != null) {
            motor.setDirection(dir);
        }
        return this;
    }

    public MotorComponent setOperationMode(MotorModes mode) {
        if (motorCurrentMode == mode) return this;

        switch (mode) {
            case Position:
                if (pidControllerForPosition == null) {
                    pidControllerForPosition = new PIDcontroller(0, 0, 0);
                }
                break;
            case AcceleratingVelocity:
                if (VPIDController == null) {
                    VPIDController = new PIDcontroller(0, 0, 0);
                }
                break;
        }
        motorCurrentMode = mode;
        return this;
    }

    // ================== Set Target ==================
    public MotorComponent setTarget(double target) {
        this.target = target;
        isOverriden = true;
        return this;
    }

    // ================== PID Coefficients ==================
    public MotorComponent setPositionCoefficients(double p, double i, double d, double zeroVelocityMultiplier) {
        if (pidControllerForPosition == null) {
            pidControllerForPosition = new PIDcontroller(p, i, d);
        } else {
            pidControllerForPosition.setConstants(p, i, d);
        }

        for (DcMotorEx motor : motorMap.values()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.zeroVelocityMultiplier = zeroVelocityMultiplier;
        return this;
    }

    public MotorComponent setAccelerationVelocityCoefficients(double p, double i, double d, double f, double s) {
        if (VPIDController == null) {
            VPIDController = new PIDcontroller(p, i, d);
        } else {
            VPIDController.setConstants(p, i, d);
        }
        vpidF = f;
        vpidS = s;
        return this;
    }

    // ================== Getters ==================
    public boolean isOverriden() { return isOverriden; }
    public double getVelocity() { return readVelocity; }
    public MotorModes getOperationMode() { return this.motorCurrentMode; }
    public double getPosition() { return readPosition / resolution; }
    public double getAbsolutePosition() { return readPosition; }
    public double getPower() { return power; }
    public double getCurrent() { return readAmps; }
    public double getError() { return target - getPosition(); }
    public DcMotorEx getMotor(String name) { return motorMap.get(name); }

    // ================== Main Loop ==================
    @Override
    public void update() {
        if (min_range <= 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }

        double targetPower = target / resolution;

        switch (motorCurrentMode) {
            case Power:
                break;

            case Position:
                targetPower = pidControllerForPosition.calculate(target, readPosition / resolution);
                if (Math.abs(readVelocity) < 0.01) {
                    targetPower *= zeroVelocityMultiplier;
                }
                break;

            case AcceleratingVelocity:
                if (target == 0) {
                    targetPower = 0;
                } else {
                    targetPower = VPIDController.calculate(target, readVelocity)
                            + target * vpidF
                            + vpidS * Math.signum(target);
                }
                break;
        }

        // Optional Voltage Compensation
        if (voltageCompensation && currentVoltage > 0) {
            targetPower = clamp(targetPower * (targetVoltage / currentVoltage), -1.0, 1.0);
        }

        power = targetPower;

        // Threshold Delta Check (Prevents Unnecessary Hardware Serial Writes)
        if (Math.abs(lastPower - power) > 0.01) {
            for (DcMotorEx motor : motorMap.values()) {
                motor.setPower(power);
            }
            lastPower = power;
        }
    }

    @Override
    public void read() {
        if (mainMotor == null) return;

        if(shouldReadPositionAndVelo) {
            // Bulk-cached position read
            readPosition = mainMotor.getCurrentPosition();

            // 0.0 ms Software Velocity calculation (Prevents hardware bus stalling)
            double dt = velocityTimer.seconds();
            if (dt >= 0.005) { // Calculate every >= 5 ms
                readVelocity = (readPosition - lastPositionForVel) / dt;
                lastPositionForVel = readPosition;
                velocityTimer.reset();
            }
        }

        // Amperage check (Only enable when strictly required)
        if (shouldReadAmps) {
            readAmps = mainMotor.getCurrent(CurrentUnit.AMPS);
        }
    }
}