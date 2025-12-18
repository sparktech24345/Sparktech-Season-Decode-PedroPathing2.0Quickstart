package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Multithread;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.hardwareMapInstance;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.PIDcontroller;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class AtomicCRServoComponent extends AtomicComponent {

    protected AtomicReference<HashMap<String, CRServo>> motorMap = new AtomicReference<>(new HashMap<>());
    protected AtomicReference<CRServo> mainServo = null;
    protected AtomicBoolean usePID = new AtomicBoolean(false);
    protected AtomicReference<PIDcontroller> PID = null;
    protected AtomicReference<PIDcontroller> PIDcamera = null;
    protected AtomicReference<Double> overridePower = new AtomicReference<>(-2.0);
    protected AtomicReference<Double> overrideTarget = new AtomicReference<>(0.0);
    protected AtomicReference<Double> lastPower = new AtomicReference<>(0.0);
    protected AtomicBoolean overrideTarget_bool = new AtomicBoolean(false);
    protected AtomicBoolean overridePower_bool = new AtomicBoolean(false);

    // Encoder for servo position feedback
    protected AtomicReference<AnalogInput> servoEncoder = null;
    protected AtomicReference<DcMotorEx> externalEncoder = null;
    protected AtomicReference<Double> externalEncoderPosition= new AtomicReference<>(0.0);
    protected AtomicReference<Double> pinpointPosition = new AtomicReference<>(0.0); // a passer value
    protected AtomicReference<Double> externalEncoderAbsolutePosition = new AtomicReference<>(0.0);
    protected AtomicReference<Double> curentPos = new AtomicReference<>(0.0);
    protected AtomicReference<Double> lastCurentPos = new AtomicReference<>(0.0);
    protected AtomicReference<Double> lastLastCurentPos = new AtomicReference<>(0.0);
    protected AtomicReference<Double> lastLastLastCurentPos = new AtomicReference<>(0.0);
    protected AtomicReference<Double> maxAccel = new AtomicReference<>(10000.0);
    protected AtomicReference<Double> maxVel = new AtomicReference<>(4000.0);
    protected AtomicReference<Double> motionTime = new AtomicReference<>(0.5);
    protected final double encoderUnitConstant = 123.37;
    protected AtomicBoolean overridePIDerror = new AtomicBoolean(false);
    protected AtomicReference<Double> errorOverride = new AtomicReference<>(0.0);

    protected AtomicReference<Double> encoderDirectionMulti = new AtomicReference<>(-1.0);
    protected AtomicReference<Double> MinErrorThreshold = new AtomicReference<>(1.2);
    protected AtomicReference<Double> ErrorThresholdMulti = new AtomicReference<>(0.0);
    protected AtomicReference<Double> MinVelocityThreshold = new AtomicReference<>(0.13);
    protected AtomicReference<Double> VelocityThresholdMulti = new AtomicReference<>(4.5);
    protected AtomicReference<Double> cameraTarget = new AtomicReference<>(0.0);

    public static double PUBLIC_encoderDirectionMulti = -1;
    public static double PUBLIC_MinErrorThreshold = 1.2;
    public static double PUBLIC_ErrorThresholdMulti = 0;
    public static double PUBLIC_MinVelocityThreshold = 0.13;
    public static double PUBLIC_VelocityThresholdMulti = 4.5;
    public static double PUBLIC_cameraTarget = 0;
    public static boolean set_all_statics = false;

    public AtomicCRServoComponent addMotor(String hardwareMapName) {
        CRServo motor = hardwareMapInstance.get(CRServo.class, hardwareMapName);
        if (motorMap.get().isEmpty()) {
            mainServo = new AtomicReference<>(motor);
        }
        motorMap.get().put(hardwareMapName, motor);
        return this;
    }
    public AtomicCRServoComponent addMotor(String hardwareMapName, String customMapName) {
        CRServo motor = hardwareMapInstance.get(CRServo.class, hardwareMapName);
        if (motorMap.get().isEmpty()) {
            mainServo = new AtomicReference<>(motor);
        }
        motorMap.get().put(customMapName, motor);
        return this;
    }

    public AtomicCRServoComponent setDirection(String servoName, CRServo.Direction dir) {
        motorMap.get().get(servoName).setDirection(dir);
        return this;
    }
    public AtomicCRServoComponent setEncoder(String analogSensorName) {
        servoEncoder = new AtomicReference<>(hardwareMapInstance.get(AnalogInput.class,analogSensorName));
        return this;
    }

    public AtomicCRServoComponent setExternalEncoder(String motorName) {
        externalEncoder = new AtomicReference<>(hardwareMapInstance.get(DcMotorEx.class,motorName));
        return this;
    }

    public AtomicCRServoComponent setPIDconstants(double p, double i, double d) {
        if (PID == null) PID = new AtomicReference<>(new PIDcontroller());
        PID.get().setConstants(p, i, d);
        return this;
    }

    public AtomicCRServoComponent setMotionconstants(double maxVel, double maxAccel, double motionTime) {
        this.maxAccel.set(maxAccel);
        this.maxVel.set(maxVel);
        this.motionTime.set(motionTime);
        return this;
    }
    public AtomicCRServoComponent setRange(double min, double max) {
        min_range.set(min);
        max_range.set(max);
        return this;
    }

    public AtomicCRServoComponent setPowerOverride(double power) {
        this.overridePower.set(power);
        return this;
    }

    public AtomicCRServoComponent setOverrideBool(boolean overrideBool) {
        this.overrideTarget_bool.set(overrideBool);
        return this;
    }

    public AtomicCRServoComponent setTargetOverride(double target) {
        this.overrideTarget.set(target);
        return this;
    }

    public AtomicCRServoComponent useWithPIDController(boolean b) {
        if (PID == null) PID = new AtomicReference<>(new PIDcontroller());
        usePID.set(b);
        return this;
    }


    public double getPosition() { return mainServo.get().getPower(); }
    public double getAnalogPosition() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.get().getVoltage() / 3.3) * 360; //should be the position in degrees, resets under 0 or above 360
    }
    public void setPinpointPosition(double pinpointPosition) {
        this.pinpointPosition.set(pinpointPosition);
    }
    public AtomicCRServoComponent initExternalEncoderPosition(double adder) {
        externalEncoderAbsolutePosition.set(externalEncoderAbsolutePosition.get() + adder); //+ getAnalogPosition();
        return this;
    }
    // No need for turn around logic due to encoder not having a limit

    //    public void updateExternalEncoderPosition(){
//        double lastPosition = externalEncoderPosition;
//        externalEncoderPosition = getEncoderReadingFormatted();
//
//        double deltaPosition = externalEncoderPosition - lastPosition;
//        if (deltaPosition > 180) {
//            deltaPosition -= 360;
//        } else if (deltaPosition < -180) {
//            deltaPosition += 360;
//        }
//
//        externalEncoderAbsolutePosition += deltaPosition;
//    }
    public double getExternalEncoderPosition() {
        return externalEncoderPosition.get();
    }

    public double averagePosition() {
        curentPos.set(pinpointPosition.get());
        lastCurentPos.set(curentPos.get());
        lastLastCurentPos.set(lastCurentPos.get());
        lastLastLastCurentPos.set(lastLastCurentPos.get());
        return (curentPos.get() + lastCurentPos.get() + lastLastCurentPos.get() + lastLastLastCurentPos.get()) / 4;
    }


    public double getPower() {
        return mainServo.get().getPower();
    }
    public double getCalculatedPower() {
        return lastPower.get();
    }

    public CRServo get(String name) {
        return motorMap.get().get(name);
    }

    public double getEncoderReadingFormatted() {
        if (externalEncoder == null) return 0;
        double reading = externalEncoder.get().getCurrentPosition() * encoderDirectionMulti.get() / encoderUnitConstant;
//        while (reading < -180) reading += 360;
//        while (reading > 180) reading -= 360;
        return reading;
    }

    public AtomicCRServoComponent setCameraTarget(double target) {
        cameraTarget.set(target);
        return this;
    }

    public double getEncoderVelocity() {
        if (externalEncoder == null) return 0;
        return externalEncoder.get().getVelocity();
    }

    public AtomicCRServoComponent overridePIDerror(double val, boolean condition) {
        errorOverride.set(val);
        overridePIDerror.set(condition);
        return this;
    }

    public AtomicCRServoComponent setCameraPIDconstants(double p, double i, double d) {
        if (PIDcamera == null) PIDcamera = new AtomicReference<>(new PIDcontroller());
        PIDcamera.get().setConstants(p, i, d);
        return this;
    }

    private double PID_camera() {
        if (PIDcamera == null) PIDcamera = new AtomicReference<>(new PIDcontroller());
        double targetPower = PIDcamera.get().calculate(cameraTarget.get(), errorOverride.get());
        if (Math.abs(errorOverride.get()) <= MinErrorThreshold.get()) targetPower *= ErrorThresholdMulti.get();
        if (Math.abs(getEncoderVelocity()) < MinVelocityThreshold.get()) targetPower *= VelocityThresholdMulti.get();
        return targetPower;
    }

    private double PID() {
        double pos = getEncoderReadingFormatted();
        double targetPower = PID.get().calculate(target.get(), pos);
        if (Math.abs(target.get() - pos) <= 1) targetPower *= 0;
        else if (Math.abs(target.get() - curentPos.get()) > 1 && Math.abs(getEncoderVelocity()) == 0) targetPower *= 3;
        return targetPower;
    }

    private void set_statics() {
        encoderDirectionMulti.set(PUBLIC_encoderDirectionMulti);
        MinErrorThreshold.set(PUBLIC_MinErrorThreshold);
        ErrorThresholdMulti.set(PUBLIC_ErrorThresholdMulti);
        MinVelocityThreshold.set(PUBLIC_MinVelocityThreshold);
        VelocityThresholdMulti.set(PUBLIC_VelocityThresholdMulti);
        cameraTarget.set(PUBLIC_cameraTarget);
    }

    @Override
    public void update() {
        // double dt = timer.seconds() - lastTime; //timer in seconds for motion profiler cuz physics
        // lastTime = timer.seconds();
        if (set_all_statics) {
            set_all_statics = false;
            set_statics();
        }

        if (min_range.get() < 0 && max_range.get() > 0) {
            target.set(clamp(target.get(), min_range.get(), max_range.get()));
        }
        if (overrideTarget_bool.get()) {
            target.set(overrideTarget.get());
        }
        double targetPower;
        if (usePID.get()) {
            //updateExternalEncoderPosition();
            /*   Fancy stuff
            //targetPower = PID.calculate(getTrapezoidPosition(target, maxVel, maxAccel, motionTime), avrg);
            //targetPower = PID.calculateSpecial(target, pinpointTotalPosition,kv,minimumPowerAdder);
            targetPower = PID.calculate(target, pinpointTotalPosition);
            if (Math.abs(target - pinpointTotalPosition) <= 2) targetPower *= 0;
//            if (Math.abs(target - pinpointTotalPosition) <= 2) {
//                targetPower += minimumPowerAdder * Math.signum(targetPower);
//            }o
             */
//            double error = target - getEncoderReadingFormatted();
//            if(Math.abs(error) > 80 ) targetPower = 1 * Math.signum(error);  // >80
//            else if(Math.abs(error) > 50 ) targetPower = 0.85 * Math.signum(error); // 50 - 80
//            else if(Math.abs(error) > 30 ) targetPower = 0.65 * Math.signum(error); // 30 - 50
//            else if(Math.abs(error) > 10 ) targetPower = 0.12 * Math.signum(error); // 10 - 30
//            else if(Math.abs(error) > 5 ) targetPower = 0.09 * Math.signum(error);  // 5 - 10
//            else targetPower = 0; // 0 -5
            if (PIDcamera == null) PIDcamera = new AtomicReference<>(new PIDcontroller());
            if (!overridePIDerror.get()) PIDcamera.get().setIntegralSum(0);
            else {
                double clamp_val = 1 / PIDcamera.get().getKi();
                PIDcamera.get().setIntegralSum(clamp(PIDcamera.get().getIntegralSum(), -clamp_val, clamp_val));
            }
            targetPower = (overridePIDerror.get() ? PID_camera() : PID());
        } else {
            targetPower = target.get() / resolution.get();
        }
        if (overridePower_bool.get()) targetPower = overridePower.get();



        if (Double.isNaN(targetPower)) targetPower = 0;
        for (CRServo servo : motorMap.get().values()) {
            servo.setPower(targetPower);
            lastPower.set(targetPower);
        }
    }
}
