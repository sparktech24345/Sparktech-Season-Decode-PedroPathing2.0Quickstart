package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexGamepad;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotState;
import org.firstinspires.ftc.teamcode.Experimental.StatesAndPositions.ColorSet;
import org.firstinspires.ftc.teamcode.pedroPathing.ITDConstants;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import java.lang.Math;



@TeleOp(name = "AutoAimTurret", group = "LinearOpMode")
public class AutoAimTurret extends LinearOpMode {
    double turretServoPos;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretServo = hardwareMap.get(Servo.class, "turretServo");
        Follower follower = ITDConstants.ITDcreateFollower(hardwareMap);

        while (opModeIsActive()) {
            turretServo.setPosition(turretServoPos);
        }
    }
}