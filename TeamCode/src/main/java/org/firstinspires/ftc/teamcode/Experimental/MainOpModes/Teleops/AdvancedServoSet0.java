package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexTelemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;

@Config
@TeleOp(name="AdvServo0", group="Debug")
public class AdvancedServoSet0 extends ComplexOpMode {

    public static class ServoInfo {
        public ServoImplEx servo;
        public double position;
        public double direction;
    }

    public static HashMap<String, ServoInfo> servos = new HashMap<>(4);
    public static String name = "";
    public static double position = 0.5;
    public static double direction = 0;

    @Override
    public void initialize() {}

    @Override
    public void update() { // loop()
        if (Objects.equals(name, "")) return;
        ServoInfo sv = servos.get(name);
        if (sv == null) sv = new ServoInfo();
        sv.servo = (ServoImplEx) publicHardwareMap.get(Servo.class, name);
        if (sv.servo == null) return;
        sv.position = position;
        sv.direction = direction;
        if (!servos.containsKey(name)) servos.put(name, sv);
        for (var srv : servos.values()) {
            srv.servo.setDirection(srv.direction < 0 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            srv.servo.setPosition(srv.position);
        }
    }

    @Override
    public void telemetry() { // loop()
        ComplexTelemetry.get().addLine("SELECTED: " + name);
        ComplexTelemetry.get().addLine("SERVO LIST");
        for (var pair : servos.entrySet()) {
            ComplexTelemetry.get().addLine("Servo: " + pair.getKey() + " -- position: " + pair.getValue().position + " -- direction: " + pair.getValue().direction);
        }
    }
}
