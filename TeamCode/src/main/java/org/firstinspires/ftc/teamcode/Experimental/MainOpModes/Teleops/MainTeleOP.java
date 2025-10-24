package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
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
                // all of the code
                if (robot.getControllerKey("A1").IsToggledOnPress) {
                    // intaking active -> A
                    robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL"));
                } else {
                    // intaking not active -> A
                    robot.addToQueue(new StateAction(true, "IntakeMotor", "OFF"));
                } if (robot.getControllerKey("Y1").ExecuteOnPress) {
                    // robot.togglePowerOff
                } if (gamepad1.right_bumper) {
                    // robot.slowdown
                } if (eval(gamepad1.left_trigger) && eval(gamepad1.right_trigger)) {
                    // robot.reverseDriveDirection
                } if (gamepad1.xWasPressed()) {
                    // shoot
                } if (gamepad1.right_bumper) {
                    // output trough intake
                }

                // ================================= DRIVER 2 ===============================================

                if(robot.getControllerKey("Y2").IsToggledOnPress) {
                    // Sorting Active
                } else {
                    // Sprting False
                } if (gamepad2.leftBumperWasPressed()){
                    // odometry false
                } if (gamepad2.rightBumperWasPressed()) {
                    // camera targeting false
                } if (gamepad2.xWasPressed()){
                    // prepare for shoot
                } if(gamepad2.aWasPressed()){
                    // take out a ball trough outtake
                } if(gamepad2.dpadLeftWasPressed()) {
                    // override portita mov
                } if(gamepad2.dpadRightWasPressed()) {
                    // override portita verde
                }
            }
        };

        robot.UseDefaultMovement();

        while (opModeInInit()) {
            robot.init_loop();
        }

        while(opModeIsActive()) {
            // loop
            robot.loop();
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
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("PurpleGateServo", new ServoComponent()
                .addMotor("purplegate")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("GreenGateServo", new ServoComponent()
                .addMotor("greengate")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TransferServo", new ServoComponent()
                .addMotor("transferservo")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TurretAngle", new ServoComponent()
                .addMotor("turretangle")
                .setResolution(360)
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
                .addState("OPEN", 35)
                .addState("CLOSED", 155, true);

        robot.getComponent("GreenGateServo")
                .addState("OPEN", 30)
                .addState("CLOSED", 256, true);

        robot.getComponent("IntakeSorterServo")
                .addState("REDIRECT_TO_PURPLE", 160)
                .addState("NEUTRAL", 0.5, true)
                .addState("REDIRECT_TO_GREEN", 30)
                .addState("BLOCK", 180);

        robot.getComponent("TransferServo")
                .addState("DOWN", 30, true)
                .addState("UP", 90);

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 325, true);

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
