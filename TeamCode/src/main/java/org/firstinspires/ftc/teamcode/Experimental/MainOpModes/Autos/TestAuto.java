package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexGamepad;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;

@Autonomous(name = "Test Auto", group = "Tests")
public class TestAuto extends OpMode {
    public static RobotController robotTester;
    private AutoRecorder recorder;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robotTester = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                controls();
                telemetry();
            }

            private void telemetry() {
                ComplexFollower.telemetry();
                queuer.telemetry();
                telemetry.update();
                telemetry.addData("AAA_controls: A1", "move fata spate");
                telemetry.addData("AAA_controls: B1", "move patrat");
                telemetry.addData("AAA_controls: X1", "move triunghi");
                telemetry.addData("AAA_controls: DPAD UP1", "move fata spate + hood movement");
                telemetry.addData("AAA_controls: DPAD DOWN1", "move patrat + hood movement");
            }

            private void controls() {
                if (ComplexGamepad.get("A1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(pose(20, 0, 0)));
                    robotTester.addToQueue(new MoveAction(pose(0, 0, 0)));
                }
                else if (ComplexGamepad.get("B1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(pose(20, 0, 0)));
                    robotTester.addToQueue(new MoveAction(pose(20, 20, 0)));
                    robotTester.addToQueue(new MoveAction(pose(0, 20, 0)));
                    robotTester.addToQueue(new MoveAction(pose(0, 0, 0)));
                }
                else if (ComplexGamepad.get("X1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(pose(10, 20, 0)));
                    robotTester.addToQueue(new MoveAction(pose(20, 0, 0)));
                    robotTester.addToQueue(new MoveAction(pose(0, 0, 0)));
                }
                else if (ComplexGamepad.get("DPAD_UP1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(pose(20, 0, 0)));
                    robotTester.addToQueue(new StateAction("TurretAngle", 270));
                    robotTester.addToQueue(new MoveAction(pose(0, 0, 0)));
                    robotTester.addToQueue(new StateAction("TurretAngle", 280));
                }
                else if (ComplexGamepad.get("DPAD_DOWN1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(pose(20, 0, 90)));
                    robotTester.addToQueue(new StateAction("TurretAngle", 270));
                    robotTester.addToQueue(new MoveAction(pose(20, 20, 180)));
                    robotTester.addToQueue(new StateAction("TurretAngle", 280));
                    robotTester.addToQueue(new MoveAction(pose(0, 20, 270)));
                    robotTester.addToQueue(new StateAction("TurretAngle", 270));
                    robotTester.addToQueue(new MoveAction(pose(0, 0, 0)));
                    robotTester.addToQueue(new StateAction("TurretAngle", 280));
                }
            }
        };
        ComponentMakerMethods.MakeComponents(robotTester);
        ComponentMakerMethods.MakeStates(robotTester);
        robotTester.init(OpModes.Autonomous);
        recorder = new AutoRecorder();
    }

    @Override
    public void init_loop() {
        robotTester.init_loop();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        recorder.update();
        robotTester.loop();
    }

    @Override
    public void stop() {
        try {
            recorder.save();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
