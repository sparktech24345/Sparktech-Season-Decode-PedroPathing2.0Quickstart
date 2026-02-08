package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.turretRotationMotorName;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.usedTargetX;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig.usedTargetY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;

@Config
@TeleOp(name = "TurretComponentTest", group = "Linear OpMode")
public class TurretComponentTest extends LinearOpMode {
    public static double p = 0.05;
    public static double d = 0.0015;
    public static double zeroPower = 1;
    public static double kv = 0.0008;
    public static double ka = 0.0001;
    public static double ks = 0.06;
    public static double testAdder = 0;
    public static boolean shouldTestEncoder = false;
    TurretComponent turret;
    /*
                .setFeedforwardCoefficients(0.0008, 0.00005, 0.04)
                .addMotor(turretRotationMotorName)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                // We set the multiplier to 1 because feedforward handles the 'kick' now
                .setPositionCoefficients(0.027, 0, 0.0015, 1)*/

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new TurretComponent();
        turret.setFeedforwardCoefficients(0.0008, 0.00005, 0.04)
                .addMotor(turretRotationMotorName)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                // We set the multiplier to 1 because feedforward handles the 'kick' now
                .setPositionCoefficients(0.027, 0, 0.0015, 1)
                // kV: velocity power, kA: acceleration burst, kStatic: friction bypass
                .setOperationMode(MotorComponent.MotorModes.Position)
                .setTarget(0)
                .setResolution(4.983)
                .setRange(-30, 330)
                .moveDuringInit(true);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            turret.setPositionCoefficients(p,0,d,zeroPower);
            turret.setFeedforwardCoefficients(kv,ka,ks);

            turret.updateRobotPose(new Pose(0,0,0));

            // Predict target (Target is at 0,0 in world space for example)
            // lookaheadSeconds should roughly match your control loop latency + motor response time
            double futureTarget = turret.calculateLookaheadTarget(usedTargetX, usedTargetY, 0.08) + testAdder;

            turret.setTarget(futureTarget);
            if(shouldTestEncoder){ turret.setOperationMode(MotorComponent.MotorModes.Power); turret.setTarget(0);}
            turret.update();

            sleep(50);

            tel.addData("POS",turret.getPosition());
            tel.addData("CURRENT",turret.getCurrent());
            tel.addData("TARGET POS",testAdder + turret.calculateLookaheadTarget(usedTargetX, usedTargetY, 0.08));
            tel.addData("SPEED", turret.getPower());
            tel.update();
        }
    }
}
