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
                .setVelocityCoefficients(-180, 0, -18, -15)
                .setAccelerationVelocityCoefficients(-0.0055,0,0,-0.0003,-0.1)
                .setOperationMode(MotorComponent.MotorModes.Velocity)
                .setTarget(0)
                .setRange(-3000,3000)
        );

        robot.makeComponent("TurretRotateMotor", new TurretComponent()
                .setFeedforwardCoefficients(0.0008, 0.0001, 0.04)
                .addMotor(turretRotationMotorName)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                // We set the multiplier to 1 because feedforward handles the 'kick' now
                .setPositionCoefficients(0.035, 0, 0.0015, 1)
                // kV: velocity power, kA: acceleration burst, kStatic: friction bypass
                .setOperationMode(MotorComponent.MotorModes.Position)
                .setTarget(0)
                .setResolution(5)
                .setRange(0, 360)
                .moveDuringInit(true)
        );


        robot.makeComponent("LeftGateServo", new ServoComponent()
                .addMotor(leftGateServoName)
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

        robot.makeComponent("TurretAngle", new ServoComponent()
                .addMotor(turretAngleServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0,1)
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
                .addState("OPEN", 72)
                .addState("CLOSED", 227,true);

        robot.getComponent("RightGateServo")
                .addState("CLOSED", 108, true)
                .addState("OPEN", 270);

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 316) // 0.88
                .addState("DEFAULT", 180, true)
                .addState("UP_MAX",18); // 0.05
    }
}
