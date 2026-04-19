package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Benchmark;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Button;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Components;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexGamepad;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.ButtonPressedEvent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.EventListener;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;

import java.io.IOException;
@Config
@Autonomous(name = "Test Auto", group = "Tests")
public class TestAuto extends ComplexOpMode {
    public static double multi = 1;
    private AutoRecorder recorder;
    ElapsedTime timer = new ElapsedTime();
    Benchmark loop_time = new Benchmark("LOOP TIME");

    @Override
    public void telemetry() {
        //ComplexFollower.telemetry();
        //queuer.telemetry();
        publicTelemetry.addData("AAA_controls: A1", "move fata spate");
        publicTelemetry.addData("AAA_controls: B1", "move patrat");
        publicTelemetry.addData("AAA_controls: X1", "move triunghi");
        publicTelemetry.addData("AAA_controls: DPAD UP1", "move fata spate + hood movement");
        publicTelemetry.addData("AAA_controls: DPAD DOWN1", "move patrat + hood movement");
        loop_time.into_telemetry(publicTelemetry);
    }

    @Override
    public void update() {
        loop_time.startTimer();
        recorder.update();
    }
    @Override
    public void initialize() {
        Components.init();
        recorder = new AutoRecorder();
        publicEventBus.subscribe(ButtonPressedEvent.class, (event) -> {
            Button b = event.getButton();
            if (b.equals(ComplexGamepad.A1.get())) {
                publicQueuer.addToQueue(new MoveAction(pose(20 * multi, 0, 0)).setName("Line p1"));
                //publicQueuer.addToQueue(new DelayAction(500));
                publicQueuer.addToQueue(new MoveAction(pose(0, 0, 0)).setName("Line p2"));
            }
            else if (b.equals(ComplexGamepad.B1.get())) {
                publicQueuer.addToQueue(new MoveAction(pose(20 * multi, 0, 0)).setName("Square p1"));
                publicQueuer.addToQueue(new MoveAction(pose(20 * multi, 20 * multi, 0)).setName("Square p2"));
                publicQueuer.addToQueue(new MoveAction(pose(0, 20, 0)).setName("Square p3"));
                publicQueuer.addToQueue(new MoveAction(pose(0, 0, 0)).setName("Square p4"));
            }
            else if (b.equals(ComplexGamepad.X1.get())) {
                publicQueuer.addToQueue(new MoveAction(pose(10 * multi, 20 * multi, 0)).setName("Triangle p1"));
                publicQueuer.addToQueue(new MoveAction(pose(20 * multi, 0, 0)).setName("Triangle p2"));
                publicQueuer.addToQueue(new MoveAction(pose(0, 0, 0)).setName("Triangle p3"));
            }
            else if (b.equals(ComplexGamepad.DPAD_UP1.get())) {
                ComplexFollower.follow(ComplexFollower.getCurrentPose().plus(new Pose(5, 0, 0)));
            }
            else if (b.equals(ComplexGamepad.DPAD_DOWN1.get())) {
                ComplexFollower.follow(ComplexFollower.getCurrentPose().plus(new Pose(-5, 0, 0)));
            }
            else if (b.equals(ComplexGamepad.DPAD_RIGHT1.get())) {
                ComplexFollower.follow(ComplexFollower.getCurrentPose().plus(new Pose(0, 5, 0)));
            }
            else if (b.equals(ComplexGamepad.DPAD_LEFT1.get())) {
                ComplexFollower.follow(ComplexFollower.getCurrentPose().plus(new Pose(0 ,-5, 0)));
            }
        });
    }

    @Override
    public void on_stop() {
        try {
            recorder.save();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
