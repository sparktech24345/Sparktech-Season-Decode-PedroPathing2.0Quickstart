package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeComponents;
import static org.firstinspires.ftc.teamcode.Experimental.ComponentMakerMethods.MakeStates;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

@TeleOp(name = "Main TeleOp Red", group = "Main")
public class MainTeleOp_Red extends MainTeleOP {

    @Override
    public void runOpMode() {
        // init
        ballCounter = 0;
        teamPipeline = 1;
        targetX = -targetX;
        currentTeamColor = TeamColor.Red;
        farStart = new Pose(-120,-27,Math.toRadians(90));

        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                robotMainLoop();
            }
        };
        robot.init(OpModes.TeleOP);
        MakeComponents(robot);
        MakeStates(robot);
        InitOtherStuff();
        robot.UseDefaultMovement();

        while (opModeInInit()) {
            robot.init_loop();
        }

        while (opModeIsActive()) {
            // loop
            robot.loop();
            passPose();
        }
        // stop
    }
}
