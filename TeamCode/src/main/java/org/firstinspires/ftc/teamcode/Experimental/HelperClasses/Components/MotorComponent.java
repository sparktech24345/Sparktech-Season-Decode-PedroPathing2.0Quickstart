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
    public static enum MotorModes{
        executeState,
        PIDToPositionWithState,
        powerOverride,
        VPIDOverride,
        PIDToPositionOverride
    }
    protected HashMap<String, DcMotorEx> motorMap = new HashMap<>();
    protected DcMotorEx mainMotor = null;
    protected PIDFCoefficients CoefficientsForVPID = null;
    protected PIDFCoefficients CoefficientsForPositionPID = null;
    protected double positionTolerance = 0;
    protected MotorModes motorCurrentMode = MotorModes.executeState;
    protected double overridePower = 0;
    protected double targetVPID = 0;
    protected double targetPositionPID = 0;
    protected double velocity = 0;

    // ================== Make motor component==================

    public MotorComponent() {
        super();
        motorCurrentMode = MotorModes.executeState;
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

    public MotorComponent loadState(String s) {
        target = states.get(s);
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

    public MotorComponent setDcMotorMode(DcMotorEx.RunMode mode){
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
    public MotorComponent setOperationMode(MotorModes mode){

        if(motorCurrentMode != mode){
            switch (mode){
                case PIDToPositionOverride: // they both do the same thing

                case PIDToPositionWithState:
                    if(CoefficientsForPositionPID != null)
                        for (DcMotorEx motor : motorMap.values()) {
                            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,CoefficientsForPositionPID);
                            motor.setTargetPositionTolerance( (int) positionTolerance);
                        }
                    else{
                        CoefficientsForPositionPID = new PIDFCoefficients(0,0,0,0);
                        for (DcMotorEx motor : motorMap.values()) {
                            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,CoefficientsForPositionPID);
                            motor.setTargetPositionTolerance( (int) positionTolerance);
                        }
                    }


                    break;

                case VPIDOverride:
                    if(CoefficientsForVPID != null)
                        for (DcMotorEx motor : motorMap.values()) {
                            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,CoefficientsForVPID);
                        }
                    else {
                        CoefficientsForVPID = new PIDFCoefficients(0, 0, 0, 0);
                        for (DcMotorEx motor : motorMap.values()) {
                            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CoefficientsForVPID);
                        }
                    }
                    break;
            }

        }

        this.motorCurrentMode = mode;
        return this;
    }

    // ================== Set target stuff ==================
    public MotorComponent setVPIDTarget(double targetVPID) {
        this.targetVPID = targetVPID;
        return this;
    }
    public MotorComponent setPositionPIDTarget(double targetPositionPID) {
        this.targetPositionPID = targetPositionPID;
        return this;
    }
    public MotorComponent setPowerOverride(double power) {
        this.overridePower = power;
        return this;
    }


    // ================== Set PID Coefficients ==================

    public MotorComponent setPositionPIDconstants(double p, double i, double d, double f,double positionTolerance) {
        CoefficientsForPositionPID = new PIDFCoefficients(p,i,d,f);
        this.positionTolerance = positionTolerance;
        for (DcMotorEx motor : motorMap.values()) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,CoefficientsForPositionPID);
            motor.setTargetPositionTolerance( (int) positionTolerance);
        }
        return this;
    }
    public MotorComponent setVPIDconstants(double p, double i, double d, double f) {
        if(CoefficientsForVPID == null) CoefficientsForVPID = new PIDFCoefficients(p,i,d,f);
        for (DcMotorEx motor : motorMap.values()) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,CoefficientsForVPID);
        }
        return this;
    }

    // ================== Getters ==================


    public DcMotorEx get(String name) {
        return motorMap.get(name);
    }

    public double getVelocity() {
        return velocity;
    }
    public MotorModes getOperationMode(){
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

        if (min_range < 0 && max_range > 0) {
            target = clamp(target, min_range, max_range);
        }
        double targetPower = target / resolution;


        switch (motorCurrentMode){

            // ================== state stuff ==================
            case executeState:
                for (DcMotorEx motor : motorMap.values()) {
                    motor.setPower(targetPower);
                }
                break;

            case PIDToPositionWithState:
                mainMotor.setTargetPosition( (int) target);
                targetPower = mainMotor.getPower();
                for (DcMotorEx motor : motorMap.values()) {
                    motor.setPower(targetPower);
                }
                break;


            // ================== override stuff ==================

            case powerOverride:
                targetPower = overridePower;
                for (DcMotorEx motor : motorMap.values()) {
                    motor.setPower(targetPower);
                }
                break;

            case PIDToPositionOverride:
                mainMotor.setTargetPosition( (int) targetPositionPID);
                targetPower = mainMotor.getPower();
                for (DcMotorEx motor : motorMap.values()) {
                    motor.setPower(targetPower);
                }
                break;

            case VPIDOverride:
                if(targetVPID != 0){
                    mainMotor.setVelocity(targetVPID); // this is so that we can have only 1 encoder per system of 1 or more engines on the same shaft
                    targetPower = mainMotor.getPower();
                    for (DcMotorEx motor : motorMap.values()) {
                        motor.setPower(targetPower);
                    }
                }
                else{
                    targetPower = 0;
                    for (DcMotorEx motor : motorMap.values()) {
                        motor.setPower(targetPower);
                    }
                }
                break;
        }
    }
}
