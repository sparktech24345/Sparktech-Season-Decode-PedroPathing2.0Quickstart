package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public static StateQueuer queuer = null;
    private List<LynxModule> allHubs;
    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();
    private HashMap<String, RobotState> states = new HashMap<>();
    private HashMap<String, Component> components = new HashMap<>();
    private DriveTrain movement = null;

    private void init_all() {
        //loop time improving thingy
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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
        // time improving thingy
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        ComplexGamepad.update();
        // untill here about 0.6 milisec with spikes up to 1.5
        ComplexFollower.update(); /// this is problem
        // up to here anywhere from 7 mls to 35


        // about 0.1 mls or lower
        queuer.update();

        //from 12 mls to spikes of 50
        for (Component c : components.values()) { /// this is big problem
            c.update();
        }

        //this and telemtry takes about 0.1
        if (DriveTrain.wasInitialized()) DriveTrain.loop();
        telemetry.update();
    }

    public void loop() {
        tickTimer.reset();
        runUpdates();
        // main loop takes under 0.5 milsec and is not the problem
        main_loop();
//        if(tickMS < 35) try {
//            Thread.sleep((long) (35 - tickMS));
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
        tickMS = tickTimer.milliseconds();
    }

    public double getExecMS() { return tickMS; }
}
