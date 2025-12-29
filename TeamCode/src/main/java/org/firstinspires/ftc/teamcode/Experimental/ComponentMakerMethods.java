package org.firstinspires.ftc.teamcode.Experimental;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

public class ComponentMakerMethods {

    /// This is the only place where these methods should exist for the main version of the robots functionality
    /// it sits in Experimental so that its the first thing you see no matter what

    public static void MakeComponents(RobotController robot) {
        robot.makeComponent("IntakeMotor", new MotorComponent()
                .addMotor(intakeMotorName)
                .setOperationMode(MotorComponent.MotorModes.Power)
                .setRange(-1, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
        );

        robot.makeComponent("TurretFlyWheelMotor", new MotorComponent()
                .addMotor(turretFlyWheelMotorUpName)
                .addMotor(turretFlyWheelMotorDownName)
                .setOperationMode(MotorComponent.MotorModes.Velocity)
                .setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER)
                .setVelocityCoefficients(180, 0,18,15)
                .setTarget(0)
                .setRange(-1, 1)
        );

        robot.makeComponent("TurretRotateMotor", new MotorComponent()
                .addMotor(turretRotationMotorName)
                .setOperationMode(MotorComponent.MotorModes.Position)
                .setTarget(0) // default middle point should be 0
                .setRange(0, 1)
                .moveDuringInit(true)
        );


        // Servos


        robot.makeComponent("IntakeSorterServo", new ServoComponent()
                .addMotor(intakeSorterServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("RightGateServo", new ServoComponent()
                .addMotor(rightGateServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("LeftGateServo", new ServoComponent()
                .addMotor(leftGateServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TurretAngle", new ServoComponent()
                .addMotor(turretAngleServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );
        ///init stuff
        robot.getServoComponent("TurretAngle")
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setTarget(50); // TODO CHANGE THIS
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


        // Servos


        robot.getComponent("RightGateServo")
                .addState("OPEN", 0, true)
                .addState("CLOSED", 150);

        robot.getComponent("LeftGateServo")
                .addState("OPEN", 0, true)
                .addState("CLOSED", 150);

        robot.getComponent("IntakeSorterServo")
                .addState("REDIRECT_TO_RIGHT", 0)
                .addState("REDIRECT_TO_LEFT", 0)
                .addState("BLOCK", 180, true);

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 0)
                .addState("DEFAULT", 50, true) // abt a middle point for safe init
                .addState("UP_MAX",80);
    }
}
