package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.eval;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.grade0Far;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.grade1Far;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class TeleOPasyncUpdates {
    public static AtomicReference<Limelight3A> limelight = new AtomicReference<>(null);
    public static AtomicBoolean run_flag = new AtomicBoolean(true);
    public static ExecutorService executor;
    public static AtomicReference<Double> loop_time = new AtomicReference<>(0.0);

    public static AtomicReference<ServoComponent> turretAngleComponent = new AtomicReference<>(null);
    public static AtomicReference<MotorComponent> turretSpinComponent = new AtomicReference<>(null);
    public static AtomicReference<CRServoComponent> turretRotationComponent = new AtomicReference<>(null);

    public static double llError() {

        LLResult llResult = limelight.get().getLatestResult();
        return llResult.getTx();
    }

    public static void init_all(AtomicReference<Limelight3A> ll3a_ref) {
        limelight = ll3a_ref;
        makeComponents();
        executor = Executors.newSingleThreadExecutor();
        executor.submit(() -> {
            while (run_flag.get()) {
                double start = System.currentTimeMillis();
                if (isTryingToFire) {
                    double distance = distToWall.get();
                    if (distance > 2.8) targetVelocity = grade0Far + distance * grade1Far;
                    else targetVelocity = 0;
                    turretSpinComponent.get().targetOverride(true);
                    turretSpinComponent.get().setTargetOverride((distance > 2.8 ? targetVelocity : turretVelocityOverride));

                    targetTurret = MainTeleOP.calculateHeadingAdjustment(currentPos.get(), targetX, targetY) + gamepad1.right_stick_x * 30 + adderTurretRotateForTests;
                    double camera_error = llError();
                    turretRotationComponent.get()
                            .setCameraPIDconstants(cameraP, cameraI, cameraD)
                            .overridePIDerror(camera_error, eval(camera_error)) // CORRECTIE PE CAMERA FLAG
                            .setPIDconstants(servoP, servoI, servoD)
                            .setMotionconstants(servoVel, servoAcel, servoTime)
                            .setOverrideBool(true)
                            .setTargetOverride(targetTurret);
                    //.setTargetOverride(0);
                }
                else {
                    turretSpinComponent.get().targetOverride(false);
                    targetVelocity = 0;
                    turretRotationComponent.get()
                            .overridePIDerror(0, false)
                            .setTargetOverride(0)
                    ; //TODO to set this to 0
                }
                loop_time.set(System.currentTimeMillis() - start);
            }
        });
        executor.shutdown();
    }

    public static void makeComponents() {
        turretSpinComponent.set(
                new MotorComponent()
                .addMotor("turretspin")
                .useWithPIDController(false)
                .setRPM_PIDCoefficients(0.005, 0.00055, 0)
                .setTargetOverride(0)
                .useWithEncoder(false)
                .setRange(-1, 1)
        );

        turretRotationComponent.set(
                new CRServoComponent()
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

        turretAngleComponent.set(
                new ServoComponent()
                .addMotor("turretangle")
                .setOverrideTarget_bool(false) //go to init pos
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        turretSpinComponent.get()
                .addState("OFF", 0, true)
                .addState("FULL", 1);
        turretAngleComponent.get()
                .addState("DOWN_MAX", 350, false) // 77 degrees looky
                .addState("UP_MAX", 192, true); // 50 degrees looky
    }

    public static void stop() {
        run_flag.set(false);
    }
}
