package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.MessageFormat;

public class Benchmark {

    public static class TimeSpec {
        private final long time_nano;
        public TimeSpec(long nanos) {
            this.time_nano = nanos;
        }

        public double get_ms() {
            return time_nano * 1e-6;
        }

        public double get_ns() {
            return time_nano;
        }

        public double get_s() {
            return time_nano * 1e-9;
        }
    }

    private long time_nano;
    private final String name;

    public Benchmark() {
        this.name = "GENERIC_BENCHMARK_NAME";
    }

    public Benchmark(String name) {
        this.name = name;
    }

    public Benchmark startTimer() {
        time_nano = System.nanoTime();
        return this;
    }

    public TimeSpec getTimer() {
        return new TimeSpec(System.nanoTime() - time_nano);
    }

    public void into_telemetry(MultipleTelemetry t) {
        t.addData(MessageFormat.format("Time spent during benchmark [{0}] (ms):", name), getTimer().get_ms());
    }

    public void into_telemetry(Telemetry t) {
        into_telemetry(new MultipleTelemetry(t));
    }
}
