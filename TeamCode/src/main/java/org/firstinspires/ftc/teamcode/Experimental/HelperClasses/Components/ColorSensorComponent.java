package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class ColorSensorComponent extends ThreadComponent {

    protected Map<String, NormalizedColorSensor> sensorMap = new HashMap<>();

    // ConcurrentHashMap allows safe concurrent reads/writes across threads
    protected Map<String, NormalizedRGBA> cachedColors = new ConcurrentHashMap<>();

    public ColorSensorComponent addSensor(String name, String hardwareMapName) {
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, hardwareMapName);
        sensorMap.put(name, sensor);
        cachedColors.put(name, new NormalizedRGBA());
        return this;
    }

    @Override
    protected void runAsync() {
        // Polls all I2C color sensors on the single background thread
        for (Map.Entry<String, NormalizedColorSensor> entry : sensorMap.entrySet()) {
            cachedColors.put(entry.getKey(), entry.getValue().getNormalizedColors());
        }
    }

    // Zero-latency RAM lookups on Main Thread
    public NormalizedRGBA getColors(String name) {
        return cachedColors.getOrDefault(name, new NormalizedRGBA());
    }

    public float getRed(String name) {
        return getColors(name).red;
    }
}