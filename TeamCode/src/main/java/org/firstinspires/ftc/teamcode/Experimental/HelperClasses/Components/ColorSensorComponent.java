package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode.publicHardwareMap;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Color;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexTelemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GenericColor;

public class ColorSensorComponent extends Component {
    protected ColorSensor sensor;
    protected Color detectedColor;
    protected GenericColor.Colors formattedColor;
    protected String name;

    public ColorSensorComponent(ColorSensor sensor, String name) {
        this.sensor = sensor;
        this.name = name;
        detectedColor = new Color(sensor.argb());
        formattedColor = detectedColor.getColor();
    }
    public ColorSensorComponent(String hardwareName) {
        this.sensor = publicHardwareMap.get(ColorSensor.class, hardwareName);
        this.name = hardwareName;
        detectedColor = new Color(sensor.argb());
        formattedColor = detectedColor.getColor();
    }

    public Color getDetectedColor() {
        return detectedColor;
    }

    public GenericColor.Colors getFormattedColor() {
        return formattedColor;
    }
    public void useSensorLight(boolean use){
        sensor.enableLed(use);
    }

    @Override
    public void update() {
        detectedColor = new Color(sensor.argb());
        formattedColor = detectedColor.getColor();
    }

    @Override
    public void telemetry() {
        ComplexTelemetry.get().addData(name + " r", detectedColor.r());
        ComplexTelemetry.get().addData(name + " g", detectedColor.g());
        ComplexTelemetry.get().addData(name + " b", detectedColor.b());
        ComplexTelemetry.get().addData(name + " a", detectedColor.a());
    }
}
