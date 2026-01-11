package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ColorSensorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;

import java.util.HashMap;
import java.util.Map;


public abstract class RobotController implements RobotControllerInterface {

    public static HardwareMap hardwareMap = null;
    public static MultipleTelemetry telemetry = null;
    public static StateQueuer queuer = null;
    
    
    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();
    private HashMap<String, RobotState> states = new HashMap<>();
    private HashMap<String, Component> components = new HashMap<>();
    private DriveTrain movement = null;

    private void init_all() {
        ComplexFollower.init(hardwareMap);
        ComplexFollower.setStartingPose(globalRobotPose);
        ComplexFollower.update();
        queuer = new StateQueuer();
        robotController = this;
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

    public Button getKey(String name) {
        return ComplexGamepad.get(name);
    }

    public Component getComponent(String componentName) {
         return components.get(componentName);
    }

    public MotorComponent getMotorComponent(String componentName) {
        return (MotorComponent) components.get(componentName);
    }

    public ServoComponent getServoComponent(String componentName) {
        return (ServoComponent) components.get(componentName);
    }

    public ColorSensorComponent getColorSensorComponent(String componentName) {
        return (ColorSensorComponent) components.get(componentName);
    }

    public CRServoComponent getCRServoComponent(String componentName) {
        return (CRServoComponent) components.get(componentName);
    }

    public Pose getCurrentPose() {
        return ComplexFollower.instance().getPose();
    }

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
        ComplexGamepad.update();
        ComplexFollower.update();
        for (Component c : components.values()) {
            if (c.moveDuringInit()) {
                c.update();
            }
        }
        telemetry.update();
        tickMS = tickTimer.milliseconds();
    }

    private void runUpdates() {
        ComplexGamepad.update(); // up to here about 0.5 milis
        ComplexFollower.update(); // up to here about 20 milis
        queuer.update(); // up to here also 20 milis
        for (Component c : components.values()) {
            c.update();
        } // aprox 40 milisec tends to 45 / 50
        if (DriveTrain.wasInitialized()) DriveTrain.loop();
        telemetry.update();
    }

    public void loop() {
        tickTimer.reset();
        runUpdates();
        main_loop();
        tickMS = tickTimer.milliseconds();
    }

    public double getExecMS() { return tickMS; }
}
