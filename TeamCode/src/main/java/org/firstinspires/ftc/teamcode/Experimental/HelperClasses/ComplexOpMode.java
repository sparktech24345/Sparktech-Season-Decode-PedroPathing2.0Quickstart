package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.EventBus;

import java.util.List;

public abstract class ComplexOpMode extends OpMode {

    public static MultipleTelemetry publicTelemetry = null;
    public static StateQueuer publicQueuer = null;
    public static EventBus publicEventBus = null;
    public static HardwareMap publicHardwareMap = null;

    protected MultipleTelemetry tel;
    protected final StateQueuer queuer = new StateQueuer();
    protected final EventBus eventBus = new EventBus();
    protected boolean debug_telemetry = true;
    protected List<LynxModule> hubs;



    public ComplexOpMode() {
        super();
        publicQueuer = queuer;
        publicEventBus = eventBus;
    }

    public void _update(Runnable main_update) {
        Benchmark loop = new Benchmark("[BENCHMARK] LOOP ITERATION").startTimer();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        Benchmark queue = new Benchmark("[BENCHMARK] QUEUER UPDATE").startTimer();
        queuer.update();
        if (debug_telemetry) queue.into_telemetry(tel);
        if (ComplexFollower.isInit()) {
            Benchmark follower = new Benchmark("[BENCHMARK] FOLLOWER UPDATE").startTimer();
            ComplexFollower.update();
            if (debug_telemetry) follower.into_telemetry(tel);
        }
        if (ComplexGamepad.isInit()) {
            Benchmark gp = new Benchmark("[BENCHMARK] FOLLOWER UPDATE").startTimer();
            ComplexGamepad.update();
            if (debug_telemetry) gp.into_telemetry(tel);
        }
        Benchmark mup = new Benchmark("[BENCHMARK] MAIN UPDATE").startTimer();
        main_update.run();
        if (debug_telemetry) mup.into_telemetry(tel);
        Benchmark tele = new Benchmark("[BENCHMARK] TELEMETRY UPDATE").startTimer();
        if (debug_telemetry) telemetry();
        if (debug_telemetry) tele.into_telemetry(tel);
        if (debug_telemetry) loop.into_telemetry(tel);
        tel.update();
    }

    @Override
    public final void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        publicHardwareMap = hardwareMap;
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        publicTelemetry = tel;
        ComplexFollower.setHardwareMap(hardwareMap);
        ComplexGamepad.init(gamepad1, gamepad2);
        _update(this::initialize);
    }

    @Override
    public final void init_loop() {
        _update(this::init_update);
    }

    @Override
    public final void loop() {
        _update(this::update);
    }

    @Override
    public final void start() {
        _update(this::on_start);
    }

    @Override
    public final void stop() {
        _update(this::on_stop);
    }

    public abstract void initialize();

    public abstract void update();

    public abstract void telemetry();

    public void init_update() {}

    public void on_start() {}

    public void on_stop() {}
};
