package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.StateQueuer.MainQueuer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Components;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.EventBus;

import java.util.List;
import java.util.function.BooleanSupplier;

public abstract class ComplexOpMode extends OpMode {

    public static HardwareMap publicHardwareMap = null;

    protected MultipleTelemetry tel;
    protected final StateQueuer queuer = new StateQueuer();
    protected List<LynxModule> hubs;


    public ComplexOpMode() {
        super();
    }

    private void benchmark_if(BooleanSupplier cond, Runnable upd, String bmname) {
        if (cond.getAsBoolean()) {
            Benchmark bm = new Benchmark(bmname).startTimer();
            upd.run();
            bm.into_telemetry(ComplexTelemetry.get());
        }
    }

    private void disableTelemetry() {
        ComplexTelemetry.disable();
    }

    private void enableTelemetry() {
        ComplexTelemetry.enable();
    }

    public void _update(Runnable main_update) {
        benchmark_if(() -> true, () -> {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            benchmark_if(() -> true, MainQueuer::update, "QUEUER UPDATE");
            benchmark_if(ComplexFollower::isInit, ComplexFollower::update, "FOLLOWER UPDATE");
            benchmark_if(ComplexGamepad::isInit, ComplexGamepad::update, "GAMEPAD UPDATE");
            benchmark_if(DriveTrain::isInit, DriveTrain::update, "DRIVETRAIN UPDATE");
            benchmark_if(() -> true, main_update, "MAIN UPDATE");
            benchmark_if(ComplexTelemetry::isInit, this::telemetry, "TELEMETRY UPDATE");
        }, "LOOP ITERATION");
        tel.update();
    }

    @Override
    public final void init() {
        tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        ComplexTelemetry.addTelemetry(telemetry);
        ComplexTelemetry.addTelemetry(FtcDashboard.getInstance().getTelemetry());
        publicHardwareMap = hardwareMap;
        ComplexFollower.setHardwareMap(hardwareMap);
        ComplexGamepad.init(gamepad1, gamepad2);
        Components.init();
        _update(this::initialize);
    }

    @Override
    public final void init_loop() {
        _update(this::init_update);
    }

    @Override
    public void loop() {
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
