package org.firstinspires.ftc.teamcode.Experimental;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.make_pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
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
                .useWithPIDController(false)
                .useWithEncoder(false)
                .setRange(-1, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
        );

        robot.makeComponent("TurretSpinMotor", new MotorComponent()
                .addMotor("turretspin")
                .useWithPIDController(false)
                .setRPM_PIDCoefficients(0.005, 0.00055, 0)
                .setTargetOverride(0)
                .useWithEncoder(false)
                .setRange(-1, 1)
        );

        robot.makeComponent("TurretRotate", new CRServoComponent()
                .addMotor("turretrotateleft")
                .setExternalEncoder("backpurple")
                .addMotor("turretrotateright")
                .initExternalEncoderPosition(0) // an initial offset so that the robots "0" is towards the intake
                .useWithPIDController(true)
                .setPIDconstants(0, 0, 0)
                .setDirection("turretrotateleft", DcMotorSimple.Direction.REVERSE)
                .setDirection("turretrotateright", DcMotorSimple.Direction.REVERSE)
                .setRange(-270, 270) // range for PID
                .moveDuringInit(false)
        );

        robot.makeComponent("IntakeSorterServo", new ServoComponent()
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
                .setOverrideTarget_bool(false) //go to init pos
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );
    }

    public static void MakeStates(RobotController robot) {
        robot.getComponent("IntakeMotor")
                .addState("OFF", 0, true)
                .addState("SLOW", 0.5)
                .addState("FULL", 1)
                .addState("FULL_REVERSE", -1)
                .addState("SLOW_REVERSE", -0.5);

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
                .addState("BLOCK", 93.816, true);

        robot.getComponent("TransferServo")
                .addState("DOWN", 27, true)
                .addState("MIDDLE", 45)
                .addState("UP", 230);

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 350, false) // 77 degrees looky
                .addState("UP_MAX", 192, true); // 50 degrees looky

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
