package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


public abstract class RobotController implements RobotControllerInterface {

    public static HardwareMap hardwareMap = null;
    public static MultipleTelemetry telemetry = null;
    protected VoltageSensor controlHubVoltageSensor;
    public static StateQueuer queuer = null;
    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private HashMap<String, RobotState> states = new HashMap<>();
    private HashMap<String, Component> components = new HashMap<>();
    private DriveTrain movement = null;
    public static double currentVoltage = 12;

    private void init_all() {
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        // REMOVED setMaximumParallelCommands(8) to eliminate RS-485 serial timeouts
//        PhotonCore.enable();
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();

        ComplexFollower.init(hardwareMap);
        ComplexFollower.setStartingPose(globalRobotPose);
        ComplexFollower.update();
        queuer = new StateQueuer();
        robotController = this;

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetryTimer.reset();
        voltageTimer.reset();
    }

    public RobotController() {
        init_all();
    }

    public RobotController(HardwareMap hmap, MultipleTelemetry telemetry, Gamepad gpad1, Gamepad gpad2) {
        hardwareMap = hmap;
        RobotController.telemetry = telemetry;
        ComplexGamepad.init(gpad1, gpad2);
        init_all();
    }

    public RobotController makeComponent(String name, Component component) {
        components.put(name, component);
        return this;
    }

    public RobotController executeNow(Action... actions) {
        for (Action action : actions)
            queuer.executeNow(action);
        return this;
    }

    public RobotController addToQueue(Action... actions) {
        for (Action action : actions)
            queuer.addAction(action);
        return this;
    }
    public RobotController clearMainQueue(){
        queuer.clearMainQueue();
        return this;
    }

    public Button getKey(String name) {
        return ComplexGamepad.get(name);
    }
    public void spitFollowerTelemetry(){ComplexFollower.telemetry();}

    public Component getComponent(String componentName) {
        return components.get(componentName);
    }

    public MotorComponent getMotorComponent(String componentName) {
        return (MotorComponent) components.get(componentName);
    }
    public TurretComponent getTurretComponent(String componentName) {
        return (TurretComponent) components.get(componentName);
    }

    public ServoComponent getServoComponent(String componentName) {
        return (ServoComponent) components.get(componentName);
    }

    public CRServoComponent getCRServoComponent(String componentName) {
        return (CRServoComponent) components.get(componentName);
    }

    public Pose getCurrentPose() {
        return ComplexFollower.instance().getPose();
    }
    public static double getDrivetrainCumulativePower(){return DriveTrain.getDrivetrainsCumulativePower();}

    public RobotController UseDefaultMovement(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        DriveTrain.init(frontLeftName, frontRightName, backLeftName, backLeftName);
        return this;
    }

    public RobotController UseDefaultMovement() {
        DriveTrain.init();
        return this;
    }

    public RobotController loadRobotState(String robotState) {
        RobotState state = states.get(robotState);
        HashMap<String, String> positions = state.getPositions();
        for (Map.Entry<String, String> entry : positions.entrySet()) {
            components.get(entry.getKey()).loadState(entry.getValue());
        }
        return this;
    }

    public void init(OpModes mode) {
        currentOpModes = mode;
    }

    public void init_loop() {
        tickTimer.reset();

        currentVoltage = controlHubVoltageSensor.getVoltage();

        ComplexGamepad.update();
        ComplexFollower.update();
        for (Component c : components.values()) {
            if (c.moveDuringInit()) {
                c.update();
            }
        }

        // Rate-limit telemetry to 30 Hz (every 33ms) to eliminate GC/Wi-Fi spikes
        if (telemetry != null && telemetryTimer.milliseconds() >= 33) {
            telemetry.update();
            telemetryTimer.reset();
        }

        tickMS = tickTimer.milliseconds();
    }

    private void runUpdates() {
        ComplexGamepad.update();
        ComplexFollower.update();
        queuer.update();

        for (Component c : components.values()) {
            c.update();
        }

        if (DriveTrain.wasInitialized()) DriveTrain.loop();
    }

    public void loop() {
        tickTimer.reset();

        // Rate-limit voltage reads every 250ms to prevent ADC hardware delays on every frame
        if (voltageTimer.milliseconds() >= 250) {
            currentVoltage = controlHubVoltageSensor.getVoltage();
            voltageTimer.reset();
        }

        runUpdates();
        main_loop();

        // Rate-limit telemetry to 30 Hz (every 33ms) to eliminate GC/Wi-Fi spikes
        if (telemetry != null && telemetryTimer.milliseconds() >= 33) {
            telemetry.update();
            telemetryTimer.reset();
        }

        tickMS = tickTimer.milliseconds();
    }

    public double getExecMS() { return tickMS; }
}