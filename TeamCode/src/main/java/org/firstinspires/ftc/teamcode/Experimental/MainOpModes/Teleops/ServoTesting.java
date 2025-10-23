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

        Servo outtakeLeftServo = hardwareMap.get(Servo.class, "turretrotateleft");
        Servo outtakeRightServo = hardwareMap.get(Servo.class, "turretrotateright");

        Servo turretAngleServo = hardwareMap.get(Servo.class, "turretangle");

        Servo intakeSortServo = hardwareMap.get(Servo.class, "intakeservo");

        Servo purpleGateServo = hardwareMap.get(Servo.class, "purplegate");
        Servo greenGateServo = hardwareMap.get(Servo.class, "greengate");

        Servo transferServo = hardwareMap.get(Servo.class, "transferservo");



        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            outtakeLeftServo.setPosition(0);
            outtakeRightServo.setPosition(0);
            intakeSortServo.setPosition(0);
            turretAngleServo.setPosition(0);
            outtakeLeftServo.setPosition(0);
            outtakeLeftServo.setPosition(0);

            telemetry.addData("servo pos",intakeServoPower);
            telemetry.update();
        }

    }
}


