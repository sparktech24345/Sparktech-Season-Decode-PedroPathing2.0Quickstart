package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoMultiple0s extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo servo0 = hardwareMap.get(Servo.class, "servo0");
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");

        while (opModeInInit()) {
            if (servo0 != null) servo0.setPosition(0);
            if (servo1 != null) servo1.setPosition(0);
            if (servo2 != null) servo2.setPosition(0);
            if (servo3 != null) servo3.setPosition(0);
        }

        while (opModeIsActive()) {
            if (servo0 != null) servo0.setPosition(0);
            if (servo1 != null) servo1.setPosition(0);
            if (servo2 != null) servo2.setPosition(0);
            if (servo3 != null) servo3.setPosition(0);
        }
    }
}
