package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Benchmark;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Color;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GenericColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;

@TeleOp(name="Color Sensor Test", group="Tests")
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        NormalizedColorSensor nsensor = hardwareMap.get(NormalizedColorSensor.class, GlobalStorage.colorSensorLeftName);
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, GlobalStorage.colorSensorLeftName);

        MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            tel.addData("is nsensor loaded?", nsensor != null);
            tel.addData("is sensor loaded?", sensor != null);
        }

        while(opModeIsActive()) {
            Benchmark normalized = new Benchmark("NORMALIZED").startTimer();
            NormalizedRGBA ncolor = nsensor.getNormalizedColors();
            normalized.into_telemetry(tel);

            Color ncol = new Color(ncolor.toColor());
            tel.addData("Norm red:", ncol.r());
            tel.addData("Norm green:", ncol.g());
            tel.addData("Norm blue:", ncol.b());
            tel.addData("Norm alpha:", ncol.a());

            Benchmark rgba = new Benchmark("RGBA").startTimer();
            int color = sensor.argb();
            rgba.into_telemetry(tel);

            Color col = new Color(color);
            tel.addData("red:", ncol.r());
            tel.addData("green:", ncol.g());
            tel.addData("blue:", ncol.b());
            tel.addData("alpha:", ncol.a());

            tel.update();
        }
    }
}
