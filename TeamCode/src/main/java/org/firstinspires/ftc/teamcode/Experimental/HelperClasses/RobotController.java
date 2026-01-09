package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import android.util.Pair;

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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


public abstract class RobotController implements RobotControllerInterface {

    public static ComplexFollower follower = null;
    public static ComplexGamepad gamepad = null;
    public static HardwareMap hardwareMap = null;
    public static MultipleTelemetry telemetry = null;
    public static StateQueuer queuer = null;
    
    
    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();
    private HashMap<String, RobotState> states = new HashMap<>();
    private HashMap<String, Component> components = new HashMap<>();
    private DriveTrain movement = null;

    private void init_all() {
        follower = new ComplexFollower(hardwareMap);
        follower.setStartingPose(globalRobotPose);
        follower.update();
        queuer = new StateQueuer();
        robotControllerInstance = this;
    }

    public RobotController() {
        init_all();
    }

    public RobotController(HardwareMap hmap, MultipleTelemetry telemetry, ComplexGamepad gamepad) {
        hardwareMap = hmap;
        RobotController.telemetry = telemetry;
        RobotController.gamepad = gamepad;
        init_all();
    }

    public RobotController(HardwareMap hmap, MultipleTelemetry telemetry, Gamepad gpad1, Gamepad gpad2) {
        hardwareMap = hmap;
        RobotController.telemetry = telemetry;
        gamepad = new ComplexGamepad(gpad1, gpad2);
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

    public Button getControllerKey(String name) {
        return gamepad.get(name);
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
    public <T extends ColorSensorComponent> T getColorSensorComponent(String componentName) {
        return (T) components.get(componentName);
    }
    public <T extends CRServoComponent> T getCRServoComponent(String componentName) {
        return (T) components.get(componentName);
    }
    public Pose getCurrentPose(){ return follower.instance().getPose();
//        return new Pose (- followerInstance.getInstance().getPose().getX(), no more reversing X due to Pedro beeing fixed
//                followerInstance.getInstance().getPose().getY(),
//                followerInstance.getInstance().getPose().getHeading());
    }
    public ComplexFollower getFollowerInstance(){return follower;}

    public RobotController UseDefaultMovement(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        movement = new DriveTrain(frontLeftName, frontRightName, backLeftName, backLeftName);
        return this;
    }

    public RobotController UseDefaultMovement() {
        movement = new DriveTrain();
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

    public void init(OpModes mode) {
        currentOpModes = mode;
    }

    public void init_loop() {
        tickTimer.reset();
        gamepad.update();
        follower.update();
        for (Component c : components.values()) {
            if (c.moveDuringInit()) {
                c.update();
            }
        }
        telemetry.update();
        tickMS = tickTimer.milliseconds();
    }

    private void runUpdates() {
        gamepad.update(); // up to here about 0.5 milis
        follower.update(); // up to here about 20 milis
        queuer.update(); // up to here also 20 milis
        for (Component c : components.values()) {
            c.update();
        } // aprox 40 milisec tends to 45 / 50
        if (movement != null) movement.loop();
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
