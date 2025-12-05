package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;

@Autonomous(name = "Decode Down Auto Blue", group = "Tests")
public class TestDecodeAutoBlue extends OpMode {
    private RobotController robot;
    private AutoRecorder recorder;
    private boolean startAuto = false;
    private boolean hasShooted = false;
    private boolean isShooting = false;
    double ballCounter =0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();

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

            private void controls() { // this will happen in a loop
                boolean isMoving = robot.getFollowerInstance().getInstance().isBusy();
                LoopChecks();
                if(startAuto) AutoSequence1();
                if(ballCounter >= 3 && isShooting == false && !isMoving) AutoShootingSequence(30,1600,65);
                // to be continued if we ever get this far
                //if(hasShooted) AutoSequence2()
            }
        };
        ComponentMakerMethods.MakeComponents(robot);
        ComponentMakerMethods.MakeStates(robot);
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
        startAuto = true;
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
        robot.addAutoPosition("pose0", 0.0, 0.0, 0.0); // Default Start Position (p0)
        robot.addAutoPosition("shooting_small_triangle", -8.03860461618018, -7.573567788431964, 0.18716639316539005); // Pose1: shooting position small triangle
        robot.addAutoPosition("hp_collect", -5.6340994046429005, 41.52871439776083, 91.91953837825105); // Pose3: HP collect
        robot.addAutoPosition("collect_row1_right", -51.43651767039862, 11.520354503721704, 89.02980145363453); // Pose4: collect first row right
        robot.addAutoPosition("collect_row1_left", -53.903058870570874, 44.08911517285926, 89.58489775767065); // Pose5: collect first row left
        robot.addAutoPosition("lever_pose", -62.716122124138785, 38.33424004982776, 0.15877285655861068); // Pose6: lever pose
        robot.addAutoPosition("collect_row2_right", -75.56306613711861, 13.0993808536079, 89.83505343602236); // Pose7: collect second row right
        robot.addAutoPosition("collect_row2_left", -76.425911009781, 36.33493738850271, 88.86076793205505); // Pose8: colect second row left
        robot.addAutoPosition("shooting_big_triangle", -74.05108594518947, -9.54259917492003, 89.92128457429376); // Pose9: shooting big triangle pose
        robot.addAutoPosition("collect_row3_right", -99.2199034202756, 14.359903410663756, 89.71481478587346); // Pose10: collect third row right
        robot.addAutoPosition("collect_row3_left", -99.35278704785925, 35.598865418922244, 90.22326772806754); // Pose11: collect third row left
        robot.addAutoPosition("start_from_sorter", -121.51062492310533, 29.559189203217276, -135.79350439863808); // Pose12: start position from sorter

// Pose2 and Pose13 were excluded based on your instructions ("invalid" and "ignore, error from start measured").
    }
    private void LoopChecks(){
        if(ballCounter == 0 && isShooting){
            shootTimer.reset();
            isShooting = false;
        }
        if(ballCounter == 0 && shootTimer.milliseconds() > 400) hasShooted = true;
    }
    private void AutoShootingSequence(double TurretRotation, double Velocity, double TurretFiringAngle){
        ///shooty shooty
        isShooting = true;
    }
    private void AutoSequence1(){
        robot.addToQueue(new MoveAction(false, "shooting_small_triangle")); // first shoot 3
        startAuto = false;
    }
    private void AutoSequence2(){
        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL")); // collecting
        robot.addToQueue(new MoveAction(true, "hp_collect")); // go to collect
    }
}
