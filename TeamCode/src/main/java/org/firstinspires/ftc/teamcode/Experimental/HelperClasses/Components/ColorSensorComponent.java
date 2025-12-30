package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.hardwareMap;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Color;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GenericColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.util.HashMap;

public class ColorSensorComponent extends Component {
    protected ColorSensor sensor;
    protected Color detectedColor;
    protected GenericColor.Colors formattedColor;

    public ColorSensorComponent(ColorSensor sensor) {
        this.sensor = sensor;
        detectedColor = new Color(sensor.argb());
        formattedColor = detectedColor.getColor();
    }
    public ColorSensorComponent(String hardwareName){
        this.sensor = hardwareMap.get(ColorSensor.class, hardwareName);
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

    public void telemetry(String name) {
        RobotController.telemetry.addData(name + " r", detectedColor.r());
        RobotController.telemetry.addData(name + " g", detectedColor.g());
        RobotController.telemetry.addData(name + " b", detectedColor.b());
        RobotController.telemetry.addData(name + " a", detectedColor.a());
    }
}
