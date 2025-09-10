package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "ServoTesting", group = "Linear OpMode")
public class ServoTesting extends LinearOpMode {
    double intakeServoPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo1 = hardwareMap.get(Servo.class, "expansionpushservo");
        Servo servo2 = hardwareMap.get(Servo.class, "controlpushservo");

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){

            if(gamepad1.x) intakeServoPower += 0.1;
            if(gamepad1.b) intakeServoPower -= 0.1;

            servo1.setPosition(intakeServoPower/300);
            servo2.setPosition(intakeServoPower/300);

            telemetry.addData("servo pos",intakeServoPower);
            telemetry.update();
        }

    }
}


