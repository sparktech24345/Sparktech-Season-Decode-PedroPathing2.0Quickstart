package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name = "ServoTesting", group = "Tests")
public class ServoTesting extends LinearOpMode {
    double intakeServoPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo outtakeLeftServo = hardwareMap.get(Servo.class, "turretrotateleft");
        Servo outtakeRightServo = hardwareMap.get(Servo.class, "turretrotateright");

        Servo turretAngleServo = hardwareMap.get(Servo.class, "turretangle");

        Servo intakeSortServo = hardwareMap.get(Servo.class, "intakeservo");

        Servo purpleGateServo = hardwareMap.get(Servo.class, "purplegate");
        Servo greenGateServo = hardwareMap.get(Servo.class, "greengate");

        Servo transferServo = hardwareMap.get(Servo.class, "transferservo");

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            outtakeLeftServo.setPosition(0);
            outtakeRightServo.setPosition(0);
            intakeSortServo.setPosition(0);
            turretAngleServo.setPosition(0);
            outtakeLeftServo.setPosition(0);
            outtakeLeftServo.setPosition(0);

            tel.addData("servo pos",intakeServoPower);
            tel.update();
        }

    }
}


