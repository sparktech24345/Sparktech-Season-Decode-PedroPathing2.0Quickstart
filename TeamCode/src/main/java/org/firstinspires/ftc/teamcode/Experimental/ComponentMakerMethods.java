package org.firstinspires.ftc.teamcode.Experimental;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.make_pair;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotState;

public class ComponentMakerMethods {

    /// This is the only place where these methods should exist for the main version of the robots functionality
    /// it sits in Experimental so that its the first thing you see no matter what

    public static void MakeComponents(RobotController robot) {
        robot.makeComponent("IntakeMotor", new MotorComponent()
                .addMotor("intakemotor")
                .setOperationMode(MotorComponent.MotorModes.executeState)
                .setRange(-1, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
        );
        robot.makeComponent("Servo", new ServoComponent(

        ));

        robot.makeComponent("TurretSpinMotor", new MotorComponent()
                .addMotor("turretspin")
                .setOperationMode(MotorComponent.MotorModes.VPIDOverride)
                .setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER)
                .setVPIDconstants(180, 0,18,15)
                .setVPIDTarget(0)
                .setRange(-1, 1)
        );

        //TODO:: De sters

        robot.makeComponent("TurretRotateServo", new ServoComponent()
                .addMotor("turretrotateleft")
                .setOperationMode(ServoComponent.ServoModes.executeState)
                .setResolution(243)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("IntakeSorterServo", new ServoComponent()
                .addMotor("intakeservo")
                .setOperationMode(ServoComponent.ServoModes.executeState)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("PurpleGateServo", new ServoComponent()
                .addMotor("purplegate")
                .setOperationMode(ServoComponent.ServoModes.executeState)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("GreenGateServo", new ServoComponent()
                .addMotor("greengate")
                .setOperationMode(ServoComponent.ServoModes.executeState)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TransferServo", new ServoComponent()
                .addMotor("transferservo")
                .setOperationMode(ServoComponent.ServoModes.executeState)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TurretAngle", new ServoComponent()
                .addMotor("turretangle")
                .setOperationMode(ServoComponent.ServoModes.executeState)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );
    }

    public static void MakeStates(RobotController robot) {
        robot.addTelemetryData("hello", "hello");
        robot.getComponent("IntakeMotor")
                .addState("OFF", 0, true)
                .addState("SLOW", 0.5)
                .addState("FULL", 1)
                .addState("FULL_REVERSE", -1)
                .addState("SLOW_REVERSE", -0.5);
        robot.getComponent("TurretRotateServo")
                .addState("MIDDLE_POINT", 121.5, true);

        robot.getComponent("TurretSpinMotor")
                .addState("OFF", 0, true)
                .addState("FULL", 1);

        robot.getComponent("PurpleGateServo")
                .addState("OPEN", 15, true)
                .addState("CLOSED", 122);

        robot.getComponent("GreenGateServo")
                .addState("OPEN", 30)
                .addState("CLOSED", 200, true);

        robot.getComponent("IntakeSorterServo")
                .addState("PUSH_TO_GREEN", 250) // push to green
                .addState("REDIRECT_TO_PURPLE", 146.64)
                .addState("REDIRECT_TO_GREEN", 35)
                .addState("PUSH_TO_PURPLE", 1)
                .addState("BLOCK", 93.816, true);

        robot.getComponent("TransferServo")
                .addState("DOWN", 20, true)
                .addState("MIDDLE", 45)
                .addState("UP", 230);

        robot.getComponent("TurretAngle")
                .addState("servoDown", 136.8, false) // 77 degrees looky
                .addState("UP_MAX", 36, true); // 50 degrees looky
    }
}
