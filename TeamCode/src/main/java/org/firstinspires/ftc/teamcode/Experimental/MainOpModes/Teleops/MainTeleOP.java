package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.make_pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotState;

@TeleOp(name="Main TeleOP", group="Main")
public class MainTeleOP extends LinearOpMode {

    private RobotController robot;

    @Override
    public void runOpMode() {
        // init

        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
            }
        };

        waitForStart();
        // start

        while(opModeIsActive()) {
            // loop

        }
        // stop
    }

    private void MakeComponents() {
        robot.makeComponent("IntakeMotor", new MotorComponent()
                .addMotor("intakemotor")
                .useWithPIDController(false)
                .useWithEncoder(false)
                .setRange(-1, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
        );

        robot.makeComponent("TurretSpinMotor", new MotorComponent()
                .addMotor("turretspin")
                .useWithPIDController(false)
                .useWithEncoder(false)
                .setRange(-1, 1)
        );

        robot.makeComponent("TurretRotate", new CRServoComponent()
                .addMotor("turretrotateleft")
                .addMotor("turretrotateright")
                .useWithPIDController(true)
                .setPIDconstants(0, 0, 0)
                .setDirection("turretrotateleft", DcMotorSimple.Direction.REVERSE)
                .setRange(-270,270) // range for PID
                .moveDuringInit(true)
        );

        robot.makeComponent("IntakeSorterServo", new MotorComponent()
                .addMotor("intakeservo")
                .setResolution(1)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("PurpleGateServo", new ServoComponent()
                .addMotor("purplegate")
                .setResolution(1)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("GreenGateServo", new ServoComponent()
                .addMotor("greengate")
                .setResolution(1)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TransferServo", new ServoComponent()
                .addMotor("transferservo")
                .setResolution(1)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TurretAngle", new ServoComponent()
                .addMotor("turretangle")
                .setResolution(1)
                .setRange(0, 1)
                .moveDuringInit(true)
        );
    }

    private void MakeStates() {
        robot.getComponent("IntakeMotor")
                .addState("OFF", 0, true)
                .addState("SLOW", 0.5)
                .addState("FULL", 1)
                .addState("FULL_REVERSE", -1)
                .addState("SLOW_REVERSE", -0.5);

        robot.getComponent("TurretSpinMotor")
                .addState("OFF", 0, true);

        robot.getComponent("PurpleGateServo")
                .addState("OPEN", 1)
                .addState("CLOSED", 0, true);

        robot.getComponent("GreenGateServo")
                .addState("OPEN", 1)
                .addState("CLOSED", 0, true);

        robot.getComponent("IntakeSorterServo")
                .addState("REDIRECT_TO_PURPLE", 0)
                .addState("NEUTRAL", 0.5, true)
                .addState("REDIRECT_TO_GREEN", 1);

        robot.getComponent("TransferServo")
                .addState("DOWN", 0, true)
                .addState("UP", 1);
        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 0, true);

        robot.addRobotState("TransferGreen", new RobotState(
                make_pair("GreenGateServo", "OPEN"),
                make_pair("TransferServo", "DOWN"),
                make_pair("IntakeMotor", "FULL")
        ));
        robot.addRobotState("TransferPurple", new RobotState(
                make_pair("PurpleGateServo", "OPEN"),
                make_pair("TransferServo", "DOWN"),
                make_pair("IntakeMotor", "FULL")
        ));
        robot.addRobotState("TransferUp", new RobotState(
                make_pair("IntakeMotor", "OFF"),
                make_pair("PurpleGateServo", "CLOSED"),
                make_pair("GreenGateServo", "CLOSED"),
                make_pair("TransferServo", "UP")
        ));
    }
}
