package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public abstract class ThreadComponent extends Component {

    // One shared single-thread executor for ALL ThreadComponents
    private static ScheduledExecutorService sharedExecutor;

    public static void initThreadPool() {
        if (sharedExecutor == null || sharedExecutor.isShutdown()) {
            sharedExecutor = Executors.newSingleThreadScheduledExecutor();
        }
    }

    public static void stopThreadPool() {
        if (sharedExecutor != null && !sharedExecutor.isShutdown()) {
            sharedExecutor.shutdownNow();
        }
    }

    public ThreadComponent startAsync(long intervalMs) {
        initThreadPool();
        sharedExecutor.scheduleWithFixedDelay(() -> {
            try {
                runAsync();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }, 0, intervalMs, TimeUnit.MILLISECONDS);
        return this;
    }

    protected abstract void runAsync();

    @Override
    public void update(){
        // we do nothing here cuz this runs on main thread
    };
    @Override
    public void read() {
        // also empty
    }
}