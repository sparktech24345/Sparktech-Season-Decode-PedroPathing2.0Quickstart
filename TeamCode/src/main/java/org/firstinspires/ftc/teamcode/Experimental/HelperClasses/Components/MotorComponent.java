package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;

@Config
public class MotorComponent extends EncodedComponent {
    public static double voltageMultiplier = 1;
    protected HashMap<String, DcMotor> motorMap = new HashMap<>();
    protected DcMotorEx mainMotor = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;
    protected PIDcontroller PIDForRPM = null;
    protected double overridePower = -2;
    protected boolean andreiOverride = false;
    protected double targetRpm = 0;
    protected boolean rpmOverride = false;
    public static double ksRPM = 0;
    public static double kvRPM = 0.00055;
    public static double kpRPM = 0.005;
    protected double velocity = 0;

    public MotorComponent setVoltage(double voltage) {
        this.voltage = voltage;
        return this;
    }

    protected double voltage=12;

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
        DcMotorEx motor = hardwareMapInstance.get(DcMotorEx.class, hardwareMapName);
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
    public void setRPMPIDconstants(double p, double i, double d) {
        if(PIDForRPM == null) PIDForRPM = new PIDcontroller();
        PIDForRPM.setConstants(p, i, d);
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
    public double calculateVoltageMultiplier(double voltage){
        return voltageMultiplier = 1.3 + (voltage - 11) * (0.9 - 1.3) / (14 - 11); // should be around 1.3 at 11V and 0.9 at 14
    }
    double current_pos;
    double last_pos = 0;
    ElapsedTime pos_timer = new ElapsedTime();
    double lastPosForRpm =0;
    double posForRpm =0;
    long timerForRpm =System.currentTimeMillis();
    private static final double TICKS_PER_REV = 28.0;
    private static final int BUFFER_SIZE = 100;
    private static double[] position_history = new double[BUFFER_SIZE];
    private static long[] time_history = new long[BUFFER_SIZE];
    private static int history_idx = 0;
    private static double measuredRPM = 0.0;
    public double MeasureRPM() {
        double current_pos = mainMotor.getCurrentPosition();

        // 1. Store the new measurement at the current index
        time_history[history_idx] = System.currentTimeMillis();
        position_history[history_idx] = current_pos;

        int oldest_idx = (history_idx + 1) % BUFFER_SIZE;

        double newest_pos = current_pos;
        long newest_timeMs = System.currentTimeMillis();

        double oldest_pos = position_history[oldest_idx];
        long oldest_timeMs = time_history[oldest_idx];


        long deltaT_ms = newest_timeMs - oldest_timeMs;
        double deltaP_ticks = newest_pos - oldest_pos;
        if (deltaT_ms > 50) {
            measuredRPM = (deltaP_ticks / TICKS_PER_REV) * (60000.0 / deltaT_ms);
        } else if (deltaT_ms <= 0) {
            measuredRPM = 0.0;
        }

        history_idx = (history_idx + 1) % BUFFER_SIZE;
        return measuredRPM;
    }

    public double getMeasuredRPM(){return measuredRPM;}
    public double CalculatePower(double target_rpm){
        return (target_rpm)/3500*calculateVoltageMultiplier(voltage-0.5);
        //return (target_rpm*errMultiplier + error*errMultiplier)/3500;
    }

    public MotorComponent setRPM_PIDCoefficients(double kpRPM, double kvRPM, double ksRPM){
        this.kvRPM = kvRPM;
        this.ksRPM = ksRPM;
        this.kpRPM = kpRPM;
        return this;
    }
    public double CalculatePowerV2(double targetVelocity){
        double error =0;
        error = targetVelocity - velocity;
        double power = ksRPM + kvRPM * velocity + kpRPM * error;
        if(targetVelocity == 0) return 0;
        return power;
    }

    public DcMotor get(String name) {
        return motorMap.get(name);
    }
    public double getVelocity(){return velocity;}

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
        //if(rpmOverride){MeasureRPM(); targetPower = CalculatePower(targetRpm);}
        if(rpmOverride) targetPower = CalculatePowerV2(targetRpm);
        for (DcMotor motor : motorMap.values()) {
            motor.setPower(targetPower);
        }
    }
}
