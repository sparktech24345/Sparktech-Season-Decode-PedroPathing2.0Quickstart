package org.firstinspires.ftc.teamcode.Experimental;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ColorSensorComponent;
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

        robot.makeComponent("TurretSpinMotor", new MotorComponent()
                .addMotor(turretFlyWheelMotorLeftName)
                .addMotor(turretFlyWheelMotorRightName)
                .setDirection(turretFlyWheelMotorLeftName, DcMotorSimple.Direction.REVERSE)
                .setOperationMode(MotorComponent.MotorModes.Velocity)
                .setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER)
                .setVelocityCoefficients(180, 0,18,15)
                .setTarget(0)
                .setRange(-1,3000)
        );

        robot.makeComponent("TurretRotateMotor", new MotorComponent()
                .addMotor(turretRotationMotorName)
                .setPositionCoefficients(0.027,0,0.0015,3)
                .setOperationMode(MotorComponent.MotorModes.Position)
                .setTarget(0) // default middle point should be 0
                .setResolution(2.62)
                .setRange(-30,330)
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
                .setRange(0.7, 0.9)
                .moveDuringInit(true)
        );

        // color sensor stuff

        robot.makeComponent("colorSensorRight",new ColorSensorComponent(colorSensorRightName));
        robot.makeComponent("colorSensorLeft",new ColorSensorComponent(colorSensorLeftName));
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
                .addState("OPEN", 180) // 0.5
                .addState("CLOSED", 33,true); // 0.09

        robot.getComponent("LeftGateServo")
                .addState("CLOSED", 155, true) // 0.43
                .addState("OPEN", 36); // 0.1

        robot.getComponent("IntakeSorterServo")
                .addState("REDIRECT_TO_RIGHT", 61) // 61
                .addState("REDIRECT_TO_LEFT", 154.8) // 154.8
                .addState("BLOCK", 108, true); // 0.17 is redirect to right 0.3 is block // 0.44 is redirect to left

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 262.8)
                .addState("DEFAULT", 288, true) // abt a middle point for safe init
                .addState("UP_MAX",324); // 0.9 is max down and 0.73 is max up
    }
}
