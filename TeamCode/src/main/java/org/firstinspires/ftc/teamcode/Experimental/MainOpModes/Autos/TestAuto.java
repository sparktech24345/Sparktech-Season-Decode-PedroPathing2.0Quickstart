package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;

@Autonomous(name = "Test Auto", group = "Tests")
public class TestAuto extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                controls();
                telemetry();
            }

            private void telemetry() {
            }

            private void controls() {
                if (gamepadInstance.get("A1").ExecuteOnPress) {
                    robot.addToQueue(new MoveAction(true, "p2"));
                    robot.addToQueue(new MoveAction(true, "p1"));
                }
            }
        };
        MakeComponents();
        MakeStates();
        MakeAutoStates();
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        recorder.update();
        robot.loop();
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
        robot.addAutoPosition("p1", 0, 0, 0);
        robot.addAutoPosition("p2", 10, 0, 0);
    }

    private void MakeComponents() {
    }

    private void MakeStates() {
    }
}
