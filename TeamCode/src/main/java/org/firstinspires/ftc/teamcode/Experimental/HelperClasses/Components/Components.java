package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.IntakeMotorStates;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.LeftGateServoStates;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.NoStates;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.RightGateServoStates;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.TiltServosStates;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.TurretAngleStates;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.TurretSpinStates;

public class Components {
    public static MotorComponent<IntakeMotorStates> IntakeMotor;
    public static MotorComponent<TurretSpinStates> TurretSpinMotor;
    public static TurretComponent<NoStates> TurretRotateMotor;
    public static ServoComponent<LeftGateServoStates> LeftGateServo;
    public static ServoComponent<RightGateServoStates> RightGateServo;
    public static ServoComponent<TurretAngleStates> TurretAngle;
    public static ServoComponent<TiltServosStates> TiltServos;

    public static void init() {
        IntakeMotor = new MotorComponent<>(new IntakeMotorStates())
                .addMotor(intakeMotorName)
                .setRange(-1, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                .setOperationMode(MotorComponent.MotorModes.Power)
        ;

        TurretSpinMotor = new MotorComponent<>(new TurretSpinStates())
                .addMotor(turretFlyWheelMotorLeftName)
                .addMotor(turretFlyWheelMotorRightName)
                .setDirection(turretFlyWheelMotorLeftName, DcMotorSimple.Direction.REVERSE)
                .setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER)
                .setVelocityCoefficients(-180, 0, -18, -15)
                .setAccelerationVelocityCoefficients(-0.0055,0,0,-0.0003,-0.1)
                .setOperationMode(MotorComponent.MotorModes.Velocity)
                .setRange(-3000, 3000)
        ;

        TurretRotateMotor = (TurretComponent<NoStates>) new TurretComponent<>(new NoStates())
                .setFeedforwardCoefficients(0.0008, 0.0001, 0.04)
                .addMotor(turretRotationMotorName)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                // We set the multiplier to 1 because feedforward handles the 'kick' now
                .setPositionCoefficients(0.035, 0, 0.0015, 1)
                // kV: velocity power, kA: acceleration burst, kStatic: friction bypass
                .setOperationMode(MotorComponent.MotorModes.Position)
                .setResolution(5)
                .setRange(0, 360)
        ;


        LeftGateServo = new ServoComponent<>(new LeftGateServoStates())
                .addMotor(leftGateServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
        ;

        RightGateServo = new ServoComponent<>(new RightGateServoStates())
                .addMotor(rightGateServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
        ;

        TurretAngle = new ServoComponent<>(new TurretAngleStates())
                .addMotor(turretAngleServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
        ;
        TiltServos = new ServoComponent<>(new TiltServosStates())
                .addMotor(rightTiltServoName)
                .addMotor(leftTiltServoName)
                .setOperationMode(ServoComponent.ServoModes.Position)
                .setResolution(360)
                .setRange(0, 1)
        ;
    }
}
