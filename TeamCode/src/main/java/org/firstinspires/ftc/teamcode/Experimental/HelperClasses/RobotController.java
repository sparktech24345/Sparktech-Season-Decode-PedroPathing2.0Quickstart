package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Component;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ComponentTypes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


public abstract class RobotController implements RobotControllerInterface {

    public static HardwareMap hardwareMap = null;
    protected VoltageSensor controlHubVoltageSensor;
    public static StateQueuer queuer = null;
    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private HashMap<String, RobotState> states = new HashMap<>();

    ///  List of hashmaps of components
    private HashMap<String, Component> noMotionComponents = new HashMap<>();
    private HashMap<String, Component> controlHubComponents = new HashMap<>();
    private HashMap<String, Component> expansionHubComponents = new HashMap<>();

    private List<HashMap<String, Component>> allComponents = Arrays.asList(
            controlHubComponents,
            expansionHubComponents,
            noMotionComponents
    );
    public static double currentVoltage = 12;

    /// ======================== telemetry stuff =========================
    public static MultipleTelemetry telemetry = null;
    private static ScheduledExecutorService telExecutor;
    private static final Map<String, Object> telemetryMap = new ConcurrentHashMap<>();
    public static void initTelemetry() {
        // Start background telemetry dispatcher at 30 Hz (33 ms)
        telExecutor = Executors.newSingleThreadScheduledExecutor();
        telExecutor.scheduleWithFixedDelay(() -> {
            if (telemetry == null) return;

            for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            telemetry.update();

            // Optional: Wipes keys after sending so unused lines auto-delete on the next 33ms cycle
//            telemetryMap.clear();
        }, 0, 40, TimeUnit.MILLISECONDS);
    }

    public static void addTelemetry(String key, Object value) {
        telemetryMap.put(key, value);
    }

    public static void stopTelemetry() {
        if (telExecutor != null && !telExecutor.isShutdown()) {
            telExecutor.shutdownNow();
        }
        telemetryMap.clear();
    }


    private void init_all() {

        initTelemetry();
        ComplexFollower.init(hardwareMap);
        ComplexFollower.setStartingPose(globalRobotPose);
        ComplexFollower.update();
        queuer = new StateQueuer();
        robotController = this;

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetryTimer.reset();
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

    public RobotController makeComponent(ComponentTypes componentType,String name, Component component){
        switch (componentType) {
            case controlHubComponent:
                controlHubComponents.put(name, component);
                break;
            case expansionHubComponent:
                expansionHubComponents.put(name, component);
                break;
            case noMotionComponent:
                noMotionComponents.put(name, component);
                break;
            default:
                somethingIsBAD = true;
                break;
        }
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
    public void spitFollowerTelemetry(){
//        ComplexFollower.telemetry();
    }

    @SuppressWarnings("unchecked")
    public <T extends Component> T getComponent(String name) {
        for (HashMap<String, Component> componentSets : allComponents) {
            Component component = componentSets.get(name);
            if (component != null) {
                return (T) component;
            }
        }
        return null; // Component not found in any group
    }



    public Pose getCurrentPose() {
        return globalRobotPose;
    }
    public static double getDrivetrainCumulativePower(){return DriveTrain.getDrivetrainsCumulativePower();}

    public RobotController UseDefaultMovement(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        DriveTrain.init(frontLeftName, frontRightName, backLeftName, backLeftName);
        return this;
    }

    public RobotController UseDefaultMovement() {
//        DriveTrain.init();
        ComplexFollower.getFollowerInstance().startTeleOpDrive(true);
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
        for (HashMap<String, Component> componentSets : allComponents) {
            for (Component component : componentSets.values()) {
                if(component.moveDuringInit()){
                    component.read();
                    component.update();
                }
            }
        }
        tickMS = tickTimer.milliseconds();
    }



    private void runUpdates() {
        for (Component c : controlHubComponents.values()) {
            c.update();
        }
        for (Component c : expansionHubComponents.values()) {
            c.update();
        }
    }
    private void runReads(){
        // included in bulk read
//        currentVoltage = PhotonCore.CONTROL_HUB.getInputVoltage(VoltageUnit.VOLTS);

        for (Component c : controlHubComponents.values()) {
            c.read();
        }

        globalRobotPose = ComplexFollower.instance().getPose();

        ComplexGamepad.update();


        for (Component c : noMotionComponents.values()) { // these might read in the update
            c.update();
        }
    }




    public void loop() {
        tickTimer.reset();


        runReads(); // input
        main_loop();
        ComplexFollower.update();
        queuer.update();
        runUpdates();
        if (DriveTrain.wasInitialized() && false) DriveTrain.loop();
        ComplexFollower.getFollowerInstance().setTeleOpDrive(-ComplexGamepad.get("LEFT_STICK_Y1").raw(),-ComplexGamepad.get("LEFT_STICK_X1").raw(),-ComplexGamepad.get("RIGHT_STICK_X1").raw());
        tickMS = tickTimer.milliseconds();
    }

    public double getExecMS() { return tickMS; }
}