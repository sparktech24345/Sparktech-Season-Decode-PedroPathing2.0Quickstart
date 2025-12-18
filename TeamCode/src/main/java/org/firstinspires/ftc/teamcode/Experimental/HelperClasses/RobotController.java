package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.isActive;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.rpmkd;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.rpmkp;

import android.util.Pair;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.EncodedComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Multithread.AtomicComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


public abstract class RobotController implements RobotControllerInterface {

    public static ComplexFollower followerInstance = null;
    public static ComplexGamepad gamepadInstance = null;
    public static HardwareMap hardwareMapInstance = null;
    public static MultipleTelemetry telemetryInstance = null;
    public static StateQueuer queuerInstance = null;
    
    
    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();
    private ColorSet_ITD currentColor = ColorSet_ITD.Undefined;
    private HashMap<String, RobotState> states = new HashMap<>();
    private HashMap<String, Component> components = new HashMap<>();
    private AtomicReference<HashMap<String, AtomicComponent>> atomic_components = new AtomicReference<>(new HashMap<>());
    private boolean useDefaultMovement = false;
    private DriveTrain movement = null;
    private ArrayList<Pair<BooleanSupplier, Runnable>> callbacks = new ArrayList<>();
    private HashMap<String, Object> telemetryList = new HashMap<>();
    private HashMap<String, Pose> autoPositions = new HashMap<>();

    private void init_all() {
       followerInstance = new ComplexFollower(hardwareMapInstance);
       followerInstance.update();
        queuerInstance = new StateQueuer();
        robotControllerInstance = this;
        followerInstance.setStartingPose(globalRobotPose);
    }

    public RobotController() {
        init_all();
    }

    public RobotController(HardwareMap hmap, MultipleTelemetry telemetry, ComplexGamepad gamepad) {
        hardwareMapInstance = hmap;
        telemetryInstance = telemetry;
        gamepadInstance = gamepad;
        init_all();
    }

    public RobotController(HardwareMap hmap, MultipleTelemetry telemetry, Gamepad gpad1, Gamepad gpad2) {
        hardwareMapInstance = hmap;
        telemetryInstance = telemetry;
        gamepadInstance = new ComplexGamepad(gpad1, gpad2);
        init_all();
    }

    public RobotController makeComponent(String name, Component component) {
        components.put(name, component);
        return this;
    }

    public RobotController makeComponent(String name, AtomicComponent component) {
        atomic_components.get().put(name, component);
        return this;
    }

    public RobotController setState(String robotState) {
        RobotState state = states.get(robotState);
        HashMap<String, String> poses = state.getPositions();
        Component comp = null;
        for (String s : poses.keySet()) {
            comp = components.get(s);
            comp.loadState(poses.get(s));
        }
        return this;
    }

    public RobotController addRobotState(String stateName, RobotState state) {
        states.put(stateName, state);
        return this;
    }

    public RobotController addToQueue(Action action) {
        queuerInstance.addAction(action);
        return this;
    }

    public RobotController addToQueue(Action... actions) {
        for (Action action : actions)
            queuerInstance.addAction(action);
        return this;
    }

    public Button getControllerKey(String name) {
        return gamepadInstance.get(name);
    }

    public Pose getAutoPose(String name) {
        return autoPositions.get(name);
    }

    public RobotController addCallback(BooleanSupplier exec, Runnable run) {
        callbacks.add(new Pair<>(exec, run));
        return this;
    }

    public <T extends Component> T getComponent(String componentName) {
        return (T) components.get(componentName);
    }
    public <T extends MotorComponent> T getMotorComponent(String componentName) {
        return (T) components.get(componentName);
    }
    public <T extends ServoComponent> T getServoComponent(String componentName) {
        return (T) components.get(componentName);
    }
    public <T extends CRServoComponent> T getCRServoComponent(String componentName) {
        return (T) components.get(componentName);
    }
    public Pose getCurrentPose() {
        return followerInstance.getInstance().getPose();
    }
    public ComplexFollower getFollowerInstance() {
        return followerInstance;
    }

    public RobotController UseDefaultMovement(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        movement = new DriveTrain(frontLeftName, frontRightName, backLeftName, backLeftName);
        return this;
    }

    public RobotController UseDefaultMovement() {
        movement = new DriveTrain();
        return this;
    }

    public RobotController addAutoPosition(String name, double x, double y, double headingInDegrees) {
        autoPositions.put(name, new Pose(x, y, Math.toRadians(headingInDegrees)));
        return this;
    }

    public RobotController addAutoPosition(String name, Pose position) {
        autoPositions.put(name, position);
        return this;
    }

    public RobotController setAutoStartingPos(String name) {
        followerInstance.getInstance().setStartingPose(autoPositions.get(name));
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

    public RobotController setDriveTrainSlowdown(double slowdown) {
        if (movement != null) movement.setSlowdown(slowdown);
        return this;
    }
    public RobotController setDirectionFlip(boolean shouldFlip) {
        if (movement != null) movement.setDirectionFlip(shouldFlip);
        return this;
    }
    public boolean getDirectionFlip() {
        return movement.getDirectionFlip();
    }

    public double getCurrentPosition(String componentName) {
        Component comp = components.get(componentName);
        if (comp instanceof EncodedComponent) {
            return ((EncodedComponent)comp).getPosition();
        }
        return comp.getPosition();
    }
    public MultipleTelemetry getTelemetryInstance(){return telemetryInstance;}

    public Encoder getComponentEncoder(String componentName) {
        Component comp = components.get(componentName);
        if (comp instanceof EncodedComponent) {
            return ((EncodedComponent)comp).getEncoderInstance();
        }
        return null;
    }

    public RobotController makeEncoder(String componentName) {
        Component comp = components.get(componentName);
        if (comp instanceof EncodedComponent) {
            ((EncodedComponent)comp).useWithEncoder(true);
        }
        return this;
    }

    public RobotController addTelemetryData(String str, Object obj) {
        telemetryList.put(str, obj);
        return this;
    }

    public RobotController addTelemetryData(String str, Supplier<?> obj) {
        telemetryList.put(str, obj);
        return this;
    }

    private ExecutorService executor = null;
    private AtomicBoolean init_loop_running = new AtomicBoolean(false);
    private AtomicBoolean loop_running = new AtomicBoolean(false);
    public void init(OpModes mode) {
        currentOpModes = mode;
        executor = Executors.newSingleThreadExecutor();
        executor.submit(() -> {
            while (init_loop_running.get()) {
                for (AtomicComponent c : atomic_components.get().values()) {
                    if (c.moveDuringInit()) {
                        c.update();
                    }
                }
            }
            while (loop_running.get()) {
                for (AtomicComponent c : atomic_components.get().values()) {
                    c.update();
                }
            }
        });
    }

    public void init_loop() {
        init_loop_running.set(true);
        gamepadInstance.update();
        followerInstance.update();
        for (Component c : components.values()) {
            if (c.moveDuringInit()) {
                c.update();
            }
        }
        showTelemetry();
    }

    private void runUpdates() {
        gamepadInstance.update(); // up to here about 0.5 milis
        followerInstance.update(); // up to here about 20 milis
        queuerInstance.update(); // up to here also 20 milis
        for (Component c : components.values()) {
            c.update();
        } // aprox 40 milisec tends to 45 / 50
        if (movement != null) movement.loop();
        showTelemetry(); // up to here about 60 milis
    }

    public void loop() {
        init_loop_running.set(false);
        loop_running.set(true);
        tickTimer.reset();
        runUpdates();
        main_loop();
        tickMS = tickTimer.milliseconds();
    }

    public void stop() {
        init_loop_running.set(false);
        loop_running.set(false);
        executor.shutdown();
    }

    public double getExecMS() { return tickMS; }

    private void showTelemetry() {
        for (Map.Entry<String, Object> entry : telemetryList.entrySet()) {
            if (entry.getValue() instanceof Supplier<?>)
                telemetryInstance.addData(entry.getKey(), ((Supplier<?>)entry.getValue()).get());
            else telemetryInstance.addData(entry.getKey(), entry.getValue());
        }
        telemetryInstance.update();
    }
}
