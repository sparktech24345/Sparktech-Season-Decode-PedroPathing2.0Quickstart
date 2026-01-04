package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorLeftName;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.colorSensorRightName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "testColorSensorLights", group = "Linear OpMode")
public class testColorSensorLights extends LinearOpMode {
    public static double servoPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorSensor sensorRight = hardwareMap.get(ColorSensor.class,colorSensorRightName);
        ColorSensor sensorLeft = hardwareMap.get(ColorSensor.class,colorSensorLeftName);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                sensorRight.enableLed(false);
                sensorLeft.enableLed(false);
            }

            if(gamepad1.b){
                sensorRight.enableLed(true);
                sensorLeft.enableLed(true);
            }
            if(gamepad1.xWasPressed()){
                gamepad1.setLedColor(0,255,0,5000);
                gamepad1.rumble(1,1,500);
            }
            if(gamepad1.yWasPressed()){
                gamepad1.setLedColor(255,0,255,5000);
                gamepad1.rumble(1,1,500);
            }

            tel.update();
        }
    }
}
