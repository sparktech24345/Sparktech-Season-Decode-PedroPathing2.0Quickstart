package org.firstinspires.ftc.teamcode.Experimental;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.vd;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.veld;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.velf;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.velp;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.vf;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.vp;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOpBlue.vs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

public class ComponentMakerMethods {

    public static double normalVoltageForBot = 12.6;
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
                .setDirection(turretFlyWheelMotorRightName, DcMotorSimple.Direction.REVERSE)
                .setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER)
                .setVelocityCoefficients(velp, 0, veld, velf)
                .setAccelerationVelocityCoefficients(vp,0,vd,vf,vs)
                .setOperationMode(MotorComponent.MotorModes.AcceleratingVelocity)
                .setTarget(0)
                .setRange(-3000,3000)
                .setVoltageCompensation(true)
                .setTargetVoltage(normalVoltageForBot)
        );

        robot.makeComponent("TurretRotateMotor", new TurretComponent()
                .setFeedforwardCoefficients(0.003, 0.00015, 0.06) // updated to be the ones from the teleop
                .addMotor(turretRotationMotorName)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                // set the multiplier to 1 because feedforward handles the 'kick' now
                .setPositionCoefficients(0.035, 0, 0.0015, 1)
                // velocity power, kA: acceleration burst, kStatic: friction bypass
                .setOperationMode(MotorComponent.MotorModes.Position)
                .setTarget(0)
                .setResolution(5)
                .setRange(0, 360)
                .setVoltageCompensation(true)
                .setTargetVoltage(normalVoltageForBot)
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
                .setRange(0.16,0.82) // check servo set multiple 0
                .moveDuringInit(true)
        );
        robot.makeComponent("TiltServos", new ServoComponent()
                .addMotor(rightTiltServoName)
                .addMotor(leftTiltServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0,1)
                .moveDuringInit(true)
        );
        robot.makeComponent("CameraRotateServo", new ServoComponent()
                .addMotor(CameraRotateServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0,1)
                .moveDuringInit(true)
        );
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
                .addState("OPEN", 232)
                .addState("CLOSED", 136.8,true);

        robot.getComponent("RightGateServo")
                .addState("CLOSED", 244.8, true)
                .addState("OPEN", 151);

        robot.getComponent("TurretAngle")
                .addState("DOWN_MAX", 273)
                .addState("DEFAULT", 180, true)
                .addState("UP_MAX",47);

        robot.getComponent("CameraRotateServo")
                .addState("MIDDLE", 0.49*360, true);

        robot.getComponent("TiltServos")
                .addState("RETRACTED", 0.85*360,true) // 0.85
                .addState("EXTENDED", 0.4*360); // 0.05
    }
}
