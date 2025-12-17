package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
@Disabled
@TeleOp(name = "Citit senzori", group = "Linear OpMode")
public class Testsenzori extends OpMode {
    private NormalizedColorSensor colorSensorGreen;
    private NormalizedColorSensor colorSensorPurple;
    private NormalizedRGBA greenSensorColors;
    private NormalizedRGBA purpleSensorColors;
    @Override
    public void init(){
        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensensor");
        colorSensorPurple = hardwareMap.get(NormalizedColorSensor.class, "purplesensor");

        telemetry.addLine("Senzori gata!");
        telemetry.update();
    }
    @Override
    public void loop(){
        MultipleTelemetry tele= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        greenSensorColors = colorSensorGreen.getNormalizedColors();
        purpleSensorColors = colorSensorPurple.getNormalizedColors();

        // Extract RGB for green sensor
        float gR = greenSensorColors.red;
        float gG = greenSensorColors.green;
        float gB = greenSensorColors.blue;

        // Extract RGB for purple sensor
        float pR = purpleSensorColors.red;
        float pG = purpleSensorColors.green;
        float pB = purpleSensorColors.blue;

        //for green sensor
        tele.addLine("Green sensor");
        tele.addData("Red",   gR);
        tele.addData("Green", gG);
        tele.addData("Blue",  gB);

        //for purple
        tele.addLine("Purple sensor");
        tele.addData("Red",   pR);
        tele.addData("Green", pG);
        tele.addData("Blue",  pB);

        tele.addLine("Detected (Green Sensor): " + detectColor(gR, gG, gB));
        tele.addLine("Detected (Purple Sensor): " + detectColor(pR, pG, pB));

        tele.update();
    }
    private String detectColor(float r, float g, float b) {
        if (r > g && r > b) return "RED";
        if (g > r && g > b) return "GREEN";
        if (b > r && b > g) return "BLUE";
        return "UNKNOWN";
    }

}
