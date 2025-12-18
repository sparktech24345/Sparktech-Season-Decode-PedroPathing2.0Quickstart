package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Multithread;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.hardwareMapInstance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.EncodedComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class AtomicMotorComponent extends AtomicEncodedComponent {
    protected AtomicReference<HashMap<String, DcMotor>> motorMap = new AtomicReference<>(new HashMap<>());
    protected AtomicReference<DcMotorEx> mainMotor = null;
    protected AtomicBoolean usePID = new AtomicBoolean(false);
    protected AtomicReference<PIDcontroller> PID = null;
    protected AtomicReference<PIDcontroller> PIDForRPM = null;
    protected AtomicReference<Double> overridePower = new AtomicReference<>(-2.0);
    protected AtomicBoolean andreiOverride = new AtomicBoolean(false);
    protected AtomicReference<Double> targetRpm = new AtomicReference<>(0.0);
    protected AtomicBoolean rpmOverride = new AtomicBoolean(false);
    
    protected AtomicReference<Double> ksRPM = new AtomicReference<>(0.0);
    protected AtomicReference<Double> kvRPM = new AtomicReference<>(0.0005);
    protected AtomicReference<Double> kpRPM = new AtomicReference<>(0.004);
    protected AtomicReference<Double> voltageMultiplier = new AtomicReference<>(1.0);
    
    public static double PUBLIC_ksRPM = 0;
    public static double PUBLIC_kvRPM = 0.0005;
    public static double PUBLIC_kpRPM = 0.004;
    public static double PUBLIC_voltageMultiplier = 1;
    public static boolean set_all_publics = false;
    
    protected AtomicReference<Double> velocity = new AtomicReference<>(0.0);

    public AtomicMotorComponent setVoltage(double voltage) {
        this.voltage = voltage;
        return this;
    }

    protected double voltage = 12;

    public AtomicMotorComponent() {
        super();
        PID = new AtomicReference<>(new PIDcontroller());
    }

    public DcMotor getMotor(String name) {
        return motorMap.get().get(name);
    }

    public AtomicMotorComponent loadState(String s) {
        target.set(states.get().get(s));
        return this;
    }

    public AtomicMotorComponent useWithEncoder(boolean useWithEncoder) {
        if (useWithEncoder) componentEncoder = new AtomicReference<>(new Encoder(mainMotor));
        return this;
    }

    public AtomicMotorComponent setResolution(double res) {
        resolution.set(res);
        return this;
    }

    public AtomicMotorComponent addMotor(String hardwareMapName) {
        DcMotorEx motor = hardwareMapInstance.get(DcMotorEx.class, hardwareMapName);
        if (motorMap.get().isEmpty()) {
            mainMotor.set(motor);
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorMap.get().put(hardwareMapName, motor);
        return this;
    }

    public double getPosition() {
        return mainMotor.get().getCurrentPosition();
    }
    public double getPower() {
        return mainMotor.get().getPower();
    }
    
    public AtomicMotorComponent setBehaviour(DcMotor.ZeroPowerBehavior zeroPower) {
        for (DcMotor motor : motorMap.get().values()) {
            motor.setZeroPowerBehavior(zeroPower);
        }
        return this;
    }

    public AtomicMotorComponent setDirection(String motorName, DcMotorSimple.Direction dir) {
        motorMap.get().get(motorName).setDirection(dir);
        return this;
    }

    public AtomicMotorComponent targetOverride(boolean rpmOverride) {
        this.rpmOverride.set(rpmOverride);
        return this;
    }
    public AtomicMotorComponent setOverrideCondition(boolean andreiOverride) {
        this.andreiOverride.set(andreiOverride);
        return this;
    }
    public AtomicMotorComponent setTargetOverride(double targetRpm) {
        this.targetRpm.set(targetRpm);
        return this;
    }
    public AtomicMotorComponent setPowerOverride(double power) {
        this.overridePower.set(power);
        return this;
    }

    public AtomicMotorComponent setPIDconstants(double p, double i, double d) {
        PID.get().setConstants(p, i, d);
        return this;
    }
    public AtomicMotorComponent setRPMPIDconstants(double p, double i, double d) {
        if(PIDForRPM == null) PIDForRPM = new AtomicReference<>(new PIDcontroller());
        PIDForRPM.get().setConstants(p, i, d);
        return this;
    }

    public AtomicMotorComponent useWithPIDController(boolean b) {
        usePID.set(b);
        return this;
    }

    public AtomicMotorComponent setRange(double min, double max) {
        min_range.set(min);
        max_range.set(max);
        return this;
    }
    public double calculateVoltageMultiplier(double voltage) {
        voltageMultiplier.set(1.3 + (voltage - 11) * (0.9 - 1.3) / (14 - 11)); // should be around 1.3 at 11V and 0.9 at 14
        return voltageMultiplier.get();
    }

    double current_pos;
    double last_pos = 0;
    ElapsedTime pos_timer = new ElapsedTime();
    double lastPosForRpm = 0;
    double posForRpm = 0;
    long timerForRpm = System.currentTimeMillis();
    private static final double TICKS_PER_REV = 28.0;
    private static final int BUFFER_SIZE = 100;
    private static double[] position_history = new double[BUFFER_SIZE];
    private static long[] time_history = new long[BUFFER_SIZE];
    private static int history_idx = 0;
    private static double measuredRPM = 0.0;

    public double MeasureRPM() {
        double current_pos = mainMotor.get().getCurrentPosition();

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

    public double getMeasuredRPM() {
        return measuredRPM;
    }

    public double CalculatePower(double target_rpm) {
        return (target_rpm) / 3500 * calculateVoltageMultiplier(voltage - 0.5);
        //return (target_rpm*errMultiplier + error*errMultiplier)/3500;
    }

    public AtomicMotorComponent setRPM_PIDCoefficients(double kpRPM, double kvRPM, double ksRPM) {
        this.kvRPM.set(kvRPM);
        this.ksRPM.set(ksRPM);
        this.kpRPM.set(kpRPM);
        return this;
    }
    public double CalculatePowerV2(double targetVelocity) {
        double error = targetVelocity - velocity.get();
        double power = ksRPM.get() + kvRPM.get() * velocity.get() + kpRPM.get() * error;
        if (targetVelocity == 0) return 0;
        return power;
    }

    public DcMotor get(String name) {
        return motorMap.get().get(name);
    }
    public double getVelocity() {
        if (componentEncoder != null) return componentEncoder.get().getEncoderPosition();
        return mainMotor.get().getVelocity();
    }
    
    protected void set_publics() {
        kpRPM.set(PUBLIC_kpRPM);
        ksRPM.set(PUBLIC_ksRPM);
        kvRPM.set(PUBLIC_kvRPM);
        voltageMultiplier.set(PUBLIC_voltageMultiplier);
    }

    @Override
    public void update() {
        if (set_all_publics) {
            set_all_publics = false;
            set_publics();
        }
        velocity.set(mainMotor.get().getVelocity());
        if (componentEncoder != null) {
            componentEncoder.get().update();
        }
        if (min_range.get() < 0 && max_range.get() > 0) {
            target.set(clamp(target.get(), min_range.get(), max_range.get()));
        }
        double targetPower = target.get() / resolution.get();
        if (usePID.get()) {
            targetPower = PID.get().calculate(target.get(), componentEncoder.get().getEncoderPosition());
        }
        if (andreiOverride.get()) targetPower = overridePower.get();
        //if(rpmOverride) {MeasureRPM(); targetPower = CalculatePower(targetRpm);}
        if (rpmOverride.get()) targetPower = CalculatePowerV2(targetRpm.get());
        for (DcMotor motor : motorMap.get().values()) {
            motor.setPower(targetPower);
        }
    }
}
