package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Encoder {
    private DcMotor motorInstance = null;
    private Servo servoInstance = null;

    public <T> Encoder(T instance) {
        if (instance instanceof DcMotorSimple) {
            this.motorInstance = (DcMotor) instance;
        }
        if (instance instanceof Servo) {
            this.servoInstance = (Servo) instance;
        }
    }

    public double getEncoderPosition() {
        if (motorInstance != null) return motorInstance.getCurrentPosition();
        return -1;
    }

    public void update() {}
}
