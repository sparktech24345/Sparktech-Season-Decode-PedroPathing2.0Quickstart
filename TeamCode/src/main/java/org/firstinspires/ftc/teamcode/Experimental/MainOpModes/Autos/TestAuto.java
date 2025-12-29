package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
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
                follower.telemetry(telemetry);
                telemetry.update();
            }

            private void controls() {
                if (gamepad.get("A1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(new Pose(20, 0, 0)));
                    robotTester.addToQueue(new MoveAction(new Pose(0, 0, 0)));
                }

                if (gamepad.get("B1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(new Pose(20, 0, 0)));
                    robotTester.addToQueue(new MoveAction(new Pose(20, 20, 0)));
                    robotTester.addToQueue(new MoveAction(new Pose(0, 20, 0)));
                    robotTester.addToQueue(new MoveAction(new Pose(0, 0, 0)));
                }

                if (gamepad.get("X1").ExecuteOnPress) {
                    robotTester.addToQueue(new MoveAction(new Pose(10, 20, 0)));
                    robotTester.addToQueue(new MoveAction(new Pose(20, 0, 0)));
                    robotTester.addToQueue(new MoveAction(new Pose(0, 0, 0)));
                }
            }
        };
        MakeComponents();
        MakeStates();
        MakeAutoStates();
        robotTester.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
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

    private void MakeAutoStates() {
        robotTester.addAutoPosition("p1", 0, 0, 0);
        robotTester.addAutoPosition("p2", 30, 10, 120);
    }

    private void MakeComponents() {
    }

    private void MakeStates() {
    }
}
