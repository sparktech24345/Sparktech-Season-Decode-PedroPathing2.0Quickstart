package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class ServoMultiple0s extends LinearOpMode {
    public static double posServo0 = 0;
    public static double posServo1 = 0;
    public static double posServo2 = 0;
    public static double posServo3 = 0;

    @Override
    public void runOpMode() {
        Servo servo0 = hardwareMap.get(Servo.class, "servo0");
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");

        while (opModeInInit()) {
            if (servo0 != null) servo0.setPosition(posServo0);
            if (servo1 != null) servo1.setPosition(posServo1);
            if (servo2 != null) servo2.setPosition(posServo2);
            if (servo3 != null) servo3.setPosition(posServo3);
        }

        while (opModeIsActive()) {
            if (servo0 != null) servo0.setPosition(posServo0);
            if (servo1 != null) servo1.setPosition(posServo1);
            if (servo2 != null) servo2.setPosition(posServo2);
            if (servo3 != null) servo3.setPosition(posServo3);
        }
    }
}
