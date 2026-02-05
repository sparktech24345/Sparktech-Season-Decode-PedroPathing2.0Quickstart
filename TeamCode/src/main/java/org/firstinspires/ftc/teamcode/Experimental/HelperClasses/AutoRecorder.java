package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import android.annotation.SuppressLint;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.autoRecorder.TimedPoseData;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class AutoRecorder {
    private ArrayList<TimedPoseData> poses = new ArrayList<>();
    private Pose pose = null;
    private ElapsedTime timer = null;
    private double lastSaveMs = 0;
    private boolean stopped = false;
    private String error_str = "All good!";

    public String getError() {return error_str;}

    public void update() {
        if (!stopped) {
            if (timer == null) { timer = new ElapsedTime(); }
            if (Math.abs(timer.milliseconds() - lastSaveMs) >= 25) {
                lastSaveMs = timer.milliseconds();
                pose = ComplexFollower.instance().getPose();
                TimedPoseData pose_data = new TimedPoseData(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()), timer.milliseconds());
                poses.add(pose_data);
            }
        }

    }

    @SuppressLint("SdCardPath")
    public void save() throws IOException {
        stopped = true;
        ObjectMapper mapper = new ObjectMapper();
            mapper.writerWithDefaultPrettyPrinter() // pretty-print
                    .writeValue(new File("/sdcard/FIRST/paths.json"), poses);
    }
}
