package org.firstinspires.ftc.teamcode.Experimental;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ColorSensorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

public class ComponentMakerMethods {


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
                .setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER)
                .setVelocityCoefficients(180, 0, 18, 15)
                .setAccelerationVelocityCoefficients(0.0055,0,0,0.00045)
                .setOperationMode(MotorComponent.MotorModes.Velocity)
                .setTarget(0)
                .setRange(-1,3000)
        );

        robot.makeComponent("TurretRotateMotor", new MotorComponent()
                .addMotor(turretRotationMotorName)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                .setPositionCoefficients(0.027, 0, 0.0015, 2)
                .setOperationMode(MotorComponent.MotorModes.Position)
                .setTarget(0)
                .setResolution(2.62)
                .setRange(-30, 330)
                .moveDuringInit(true)
        );



        robot.makeComponent("IntakeSorterServo", new ServoComponent()
                .addMotor(intakeSorterServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("LeftGateServo", new ServoComponent()
                .addMotor(rightGateServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("RightGateServo", new ServoComponent()
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


        robot.makeComponent("colorSensorRight", new ColorSensorComponent(colorSensorRightName));
        robot.makeComponent("colorSensorLeft", new ColorSensorComponent(colorSensorLeftName));
    }

    public static void MakeStates(RobotController robot) {
        robot.getComponent("IntakeMotor")
                .addState("OFF", 0, true)
                .addState("SLOW", 0.5)
                .addState("FULL", 1)
                .addState("FIRING_POWER", 1)
                .addState("FULL_REVERSE", -1)
                .addState("SLOW_REVERSE", -0.5);

        robot.getComponent("TurretSpinMotor")
                .addState("OFF", 0, true)
                .addState("FULL", 1);


        robot.getComponent("LeftGateServo")
                .addState("OPEN", 180)
                .addState("CLOSED", 33,true);

        robot.getComponent("RightGateServo")
                .addState("CLOSED", 155, true)
                .addState("OPEN", 30);

        robot.getComponent("IntakeSorterServo")
                .addState("REDIRECT_TO_RIGHT", 61,true)
                .addState("REDIRECT_TO_LEFT", 170)
                .addState("BLOCK", 108);

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 262.8)
                .addState("DEFAULT", 288, true)
                .addState("UP_MAX",324);
    }
}
