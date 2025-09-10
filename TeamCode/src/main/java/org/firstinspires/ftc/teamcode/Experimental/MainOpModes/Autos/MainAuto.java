package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

@Disabled
@Autonomous(name = "Main Auto", group = "Experimental")
public class MainAuto extends OpMode {
    private RobotController robot;
    private Follower follower;
    private Path pathToFollow;
    private Pose targetPos = new Pose(5,5,25);
    private MultipleTelemetry tele;

    @Override
    public void init() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2,false) {
            @Override
            public void main_loop() {

            }

        };
        MakeComponents();
        MakeStates();
        robot.init(OpModes.TeleOP);
        robot.loop();


        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        tele = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
        MakeAutoStates();
        pathToFollow = new Path(new BezierLine(follower.getPose(), targetPos));
        pathToFollow.setLinearHeadingInterpolation(follower.getPose().getHeading(),targetPos.getHeading());
        follower.followPath(pathToFollow);
    }

    @Override
    public void loop() {
        robot.loop();
        follower.update();

    }

    @Override
    public void stop() {
    }
    private void MakeAutoStates(){

        //robot.addToQueue(new MoveAction(false,new Pose(5,5,0)));
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
                .setRange(1)
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






        /*
        robot
            .makeComponent("INTAKE_EXTENSION", new MotorComponent()
                    .addMotor(intakeExtendName)
                    .useWithPIDController(true)
                    .setPIDconstants(0.009, 0.06691449814126393, 0.000302625)
                    .setRange(585)
                    .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                    .setDirection(intakeExtendName, DcMotorSimple.Direction.REVERSE)
            )
            .makeComponent("INTAKE_PIVOT", new ServoComponent()
                    .addMotor(intakePosName)
                    .setResolution(228)
            )
            .makeComponent("INTAKE_SPIN", new MotorComponent()
                    .addMotor(intakeSpinName)
                    .useWithPIDController(false)
                    .setRange(1)
                    .setDirection(intakeSpinName, DcMotorSimple.Direction.REVERSE)
            )
            .makeComponent("OUTTAKE_EXTENSION", new MotorComponent()
                    .addMotor(outtakeExtendLeftName)
                    .addMotor(outtakeExtendRightName)
                    .useWithPIDController(true)
                    .setPIDconstants(0.0105, 0.06691449814126393, 0.000112875)
                    .setRange(2100)
                    .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                    .setDirection(outtakeExtendRightName, DcMotorSimple.Direction.REVERSE)
            )
            .makeComponent("OUTTAKE_ARM", new ServoComponent()
                    .addMotor(outtakeArmName)
                    .setResolution(328)
                    .moveDuringInit(true)
            )
            .makeComponent("OUTTAKE_CLAW", new ServoComponent()
                    .addMotor(outtakeClawName)
                    .setResolution(360)
                    .moveDuringInit(true)
            );

         //*/
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

        /*
        robot.getComponent("INTAKE_EXTENSION")
                .addState("EXTENDED1", 112)
                .addState("EXTENDED2", 245)
                .addState("EXTENDED3", 377)
                .addState("EXTENDED4", 585)
                .addState("RETRACTED", 0)
                .addState("ABSOLUTE_ZERO", 0, true);

        robot.getComponent("INTAKE_PIVOT")
                .addState("SAMPLE_PICKUP", 211)
                .addState("SPIT_OUT", 15, true)
                .addState("TRANSFER_POS", 153)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("INTAKE_SPIN")
                .addState("COLLECT", 1)
                .addState("SPIT_OUT", -0.5)
                .addState("OFF", 0, true)
                .addState("SPIT_OUT_FAST", -1)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_EXTENSION")
                .addState("TRANSFER_POS", 0, true)
                .addState("PARKED", 655)
                .addState("WALL_PICKUP", 710)
                .addState("MAX_LOW_BASKET", 800)
                .addState("AUTO_SPECIMEN_HANG", 950)
                .addState("STANDBY", 1000)
                .addState("SPECIMEN_HANG", 1100)
                .addState("MAX_HIGH_BASKET", 2100)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_ARM")
                .addState("WALL_PICKUP", 285)
                .addState("HIGH_RUNG", 170)
                .addState("BASKET_SCORE", 43)
                .addState("TRANSFER_POS", 201)
                .addState("STANDBY_POS", 170, true)
                .addState("PARKED_POS", 164.84)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_CLAW")
                .addState("EXTRA_OPEN", 208)
                .addState("OPEN", 128)
                .addState("CLOSED", 35, true)
                .addState("ABSOLUTE_ZERO", 0);

        robot
                .addRobotState("TRANSFER_POS", new RobotState(
                        make_pair("INTAKE_EXTENSION", "RETRACTED"),
                        make_pair("INTAKE_PIVOT", "TRANSFER_POS"),
                        make_pair("OUTTAKE_EXTENSION", "TRANSFER_POS"),
                        make_pair("OUTTAKE_ARM", "TRANSFER_POS")
                ))
                .addRobotState("SPECIMEN_HANG", new RobotState(
                        make_pair("OUTTAKE_EXTENSION", "SPECIMEN_HANG"),
                        make_pair("OUTTAKE_ARM", "HIGH_RUNG")
                ))
                .addRobotState("STANDBY", new RobotState(
                        make_pair("INTAKE_EXTENSION", "RETRACTED"),
                        make_pair("OUTTAKE_EXTENSION", "STANDBY"),
                        make_pair("OUTTAKE_ARM", "STANDBY_POS")
                ));
//*/
    }
}
