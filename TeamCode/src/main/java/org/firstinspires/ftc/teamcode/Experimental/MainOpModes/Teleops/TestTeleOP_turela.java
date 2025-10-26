package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.make_pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotState;

@TeleOp(name="Test TeleOP Turela", group="Main")
public class TestTeleOP_turela extends LinearOpMode {

    private RobotController robot;

    @Override
    public void runOpMode() {
        // init

        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                // all of the code
                if (gamepad1.xWasPressed()) {
                    //.setPowerOverrid(True);
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

        robot.makeComponent("TurretSpinMotor", new MotorComponent()
                .addMotor("turretspin")
                .useWithPIDController(false)
                .useWithEncoder(false)
                .setRange(-1, 1)
        );

        robot.makeComponent("TurretRotate", new CRServoComponent()
                .addMotor("turretrotateleft")
//                .addMotor("turretrotateright")
                .useWithPIDController(true)
                .setPIDconstants(0, 0, 0)
                .setDirection("turretrotateleft", DcMotorSimple.Direction.REVERSE)
                .setRange(-270,270) // range for PID
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
        robot.getComponent("TurretSpinMotor")
                .addState("OFF", 0, true);


        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 325, true);

        robot.getComponent("TurretRotate")
                .addState("MIN", 0, true)
                .addState("MAX", 180, false);
    }
}
