package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;

@Disabled
@Config
@TeleOp(name = "Test PID turela", group = "Linear OpMode")
public class Test_Turela_Separat extends LinearOpMode {
    private DcMotorEx encoder = null;

    public static double p = 0.015;
    public static double d = 0;
    public static double pow = 0;
    public static double targetServoPos = 0;
    public static double maxPower = 0.4;

    public static double encoderDirectionMulti = 1;
    protected final double encoderUnitConstant = 123.37;

    @Override
    public void runOpMode(){
        CRServo servo1 = hardwareMap.get(CRServo.class, "turretrotateleft");
        CRServo servo2 = hardwareMap.get(CRServo.class, "turretrotateright");

        MultipleTelemetry tele= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        encoder = hardwareMap.get(DcMotorEx.class, "backpurple");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            //servo1.setPower(gamepad1.left_stick_x + pow);
            //servo2.setPower(gamepad1.left_stick_x + pow);

            double curentPos = encoder.getCurrentPosition()  * encoderDirectionMulti / encoderUnitConstant;
            double curentVel = encoder.getVelocity() * encoderDirectionMulti / encoderUnitConstant;

            double calculatedPower = p * (targetServoPos - curentPos) + clamp(d * curentVel,-maxPower, maxPower);

            if(Math.abs((targetServoPos - curentPos)) > 1.5 && Math.abs(curentVel) < 1) calculatedPower *= 3;
            if(Math.abs((targetServoPos - curentPos)) < 1.5) calculatedPower = 0;

            servo1.setPower(calculatedPower);
            servo2.setPower(calculatedPower);

            tele.addData("servo 1 powe",servo1.getPower());
            tele.addData("servo 2 powe",servo2.getPower());

            tele.addData("encoder position", curentPos);
            tele.addData("encoder velocity", curentVel);


            tele.update();
        }
    }
}
