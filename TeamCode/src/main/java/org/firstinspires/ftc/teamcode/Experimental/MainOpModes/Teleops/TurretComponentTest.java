package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Components.TurretRotateMotor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.TurretRotateStates;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.Configuration;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.ComponentsStates.NoStates;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.TurretComponent;

@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "TurretComponentTest", group = "Linear OpMode")
public class TurretComponentTest extends LinearOpMode {
    public static double p = 0.05;
    public static double d = 0.0015;
    public static double zeroPower = 1;
    public static double kv = 0.0008;
    public static double ka = 0.0001;
    public static double ks = 0.06;
    public static double testAdder = 0;
    public static boolean shouldTestEncoder = true;
    public static Configuration cfg = Configuration.getConfig("blue");
    /*
                .setFeedforwardCoefficients(0.0008, 0.00005, 0.04)
                .addMotor(TurretRotateMotorRotationMotorName)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                // We set the multiplier to 1 because feedforward handles the 'kick' now
                .setPositionCoefficients(0.027, 0, 0.0015, 1)*/

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TurretRotateMotor.setPositionCoefficients(p, 0, d, zeroPower);
            TurretRotateMotor.setFeedforwardCoefficients(kv, ka, ks);

            TurretRotateMotor.updateRobotPose(new Pose(0,0,0));

            // Predict target (Target is at 0,0 in world space for example)
            // lookaheadSeconds should roughly match your control loop latency + motor response time
            double futureTarget = TurretRotateMotor.calculateLookaheadTarget(cfg.usedTargetX, cfg.usedTargetY, 0.08) + testAdder;

            TurretRotateMotor.setState(futureTarget);
            if(shouldTestEncoder){ TurretRotateMotor.setOperationMode(MotorComponent.MotorModes.Power); TurretRotateMotor.setState(0);}
            TurretRotateMotor.update();

            sleep(50);

            tel.addData("POS",TurretRotateMotor.getPosition());
            tel.addData("Absolute POS",TurretRotateMotor.getAbsolutePosition());
            tel.addData("CURRENT",TurretRotateMotor.getCurrent());
            tel.addData("TARGET POS",testAdder + TurretRotateMotor.calculateLookaheadTarget(cfg.usedTargetX, cfg.usedTargetY, 0.08));
            tel.addData("SPEED", TurretRotateMotor.getPower());
            tel.update();
        }
    }
}
