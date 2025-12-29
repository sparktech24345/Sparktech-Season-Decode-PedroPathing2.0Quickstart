package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.AutoRecorder;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.IOException;

@Config
@Autonomous(group="park only", name="park only blue small triangle")
public class AutoParkOnly extends OpMode {

    RobotController robot = null;
    AutoRecorder recorder;

    public Pose getParkPose() {
        return new Pose(15, 15, 0);
    }
    public Pose getStartingPose() {
        return new Pose(0, 0, 0);
    }

    @Override
    public void init() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                RobotController.telemetry.addData("robot X", robot.getCurrentPose().getX());
                RobotController.telemetry.addData("robot Y", robot.getCurrentPose().getY());
                RobotController.telemetry.addData("robot rotation", Math.toDegrees(robot.getCurrentPose().getHeading()));
            }
        };
        robot.init(OpModes.Autonomous);
        recorder = new AutoRecorder(true);
        robot.getFollowerInstance().setStartingPose(getStartingPose());
    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
        robot.addToQueue(new MoveAction(getParkPose()));
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
        GlobalStorage.globalRobotPose = robot.getCurrentPose();
    }
}
