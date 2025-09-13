package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.geometry.*;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

@Disabled
@Autonomous(name = "Main Auto", group = "Experimental")
public class MainAuto extends OpMode {
    private RobotController robot;

    @Override
    public void init() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {

            }
        };
        MakeComponents();
        MakeStates();
        robot.init(OpModes.TeleOP);
    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
        MakeAutoStates();
    }

    @Override
    public void loop() {
        robot.loop();
    }

    @Override
    public void stop() {
    }

    private void MakeAutoStates() {

        robot.addToQueue(new MoveAction(false,new Pose(5,5,0)));
        robot.addToQueue(new DelayAction(false, 1200));
        //trowing first
        robot.addToQueue(new StateAction(true, "LAUNCHING", "OUTPUT"));
        robot.addToQueue(new DelayAction(true, 200));
        robot.addToQueue(new StateAction(true, "PUSH_SERVO", "BALL_IN"));
        robot.addToQueue(new DelayAction(true, 80));
        robot.addToQueue(new StateAction(true, "PUSH_SERVO", "WAITING_BALL"));
        robot.addToQueue(new DelayAction(true, 300));
        robot.addToQueue(new StateAction(true, "LAUNCHING", "OFF"));
        robot.addToQueue(new StateAction(true, "PUSH_SERVO", "PARTIAL_POS"));

        // arming second
        robot.addToQueue(new DelayAction(true, 50));
        robot.addToQueue(new StateAction(true, "HOLD_SERVO", "EXTEND_POS"));
        robot.addToQueue(new DelayAction(true, 450));
        //shooting second

        robot.addToQueue(new StateAction(true, "LAUNCHING", "OUTPUT"));
        robot.addToQueue(new DelayAction(true, 200));
        robot.addToQueue(new StateAction(true, "PUSH_SERVO", "BALL_IN"));
        robot.addToQueue(new DelayAction(true, 80));
        robot.addToQueue(new StateAction(true, "PUSH_SERVO", "WAITING_BALL"));
        robot.addToQueue(new DelayAction(true, 300));
        robot.addToQueue(new StateAction(true, "LAUNCHING", "OFF"));
        robot.addToQueue(new StateAction(true, "PUSH_SERVO", "PARTIAL_POS"));

    }

    private void MakeComponents() {
        robot.makeComponent("LAUNCHING", new MotorComponent()
                .addMotor(LaunchMotorOneName)
                .addMotor(LaunchMotorTwoName)
                .useWithPIDController(false)
                .setRange(0, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(LaunchMotorOneName, DcMotorSimple.Direction.REVERSE)
        );
        robot.makeComponent("PUSH_SERVO", new ServoComponent()
                .addMotor(PushServoOneName)
                .setResolution(300)
        );

        robot.makeComponent("HOLD_SERVO", new ServoComponent()
                .addMotor(PushServoTwoName)
                .setResolution(300)
        );
    }

    private void MakeStates() {
        robot.getComponent("LAUNCHING")
                .addState("OUTPUT", 1)
                .addState("INPUT", -1)
                .addState("OFF", 0, true)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("PUSH_SERVO")
                .addState("BALL_IN", 110)
                .addState("PARTIAL_POS",20,true)
                .addState("WAITING_BALL", 0)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("HOLD_SERVO")
                .addState("EXTEND_POS", 150)
                .addState("MIDDLE_POS",50,true)
                .addState("LOADING", 0)
                .addState("ABSOLUTE_ZERO", 0);
    }
}
