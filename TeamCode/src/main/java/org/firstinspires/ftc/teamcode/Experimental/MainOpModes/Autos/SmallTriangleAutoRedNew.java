package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentTeamColor;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.teamPipeline;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.TeamColor;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Autonomous(name = "Small Triangle Auto RED",group = "AAA")
public class SmallTriangleAutoRedNew extends SmallTriangleNew {
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
    @Override
    public Pose convertPose(Pose pose) {
        return new Pose(pose.getX(), -pose.getY(), -pose.getHeading());
    }
    @Override
    public double getBallNumber(){
        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null) {
            double[] pythonData = llResult.getPythonOutput();
            if (pythonData.length > 0) {
                double firstValue = pythonData[0];
                RobotController.telemetry.addData("Python Val 1", firstValue);

                if(firstValue == 1) firstValue = 3;
                firstValue--; // this way if its one then its 2, and if its 2 then its one

                return firstValue;
            }
        }
        return 0;
    }
}