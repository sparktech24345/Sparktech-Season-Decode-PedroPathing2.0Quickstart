package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import android.annotation.SuppressLint;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.geometry.Pose;
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
    private boolean always_run = true;
    private boolean stopped = false;
    private BooleanSupplier path_saving;
    private BooleanSupplier pose_adding;
    private String error_str = "All good!";

    public AutoRecorder(boolean always_run) {
        this.always_run = always_run;
    }

    public void setRunMode(boolean always_run) {
        this.always_run = always_run;
    }

    public String getError() {return error_str;}

    public void update() {
        if (!stopped) {
            if (timer == null) { timer = new ElapsedTime(); timer.reset(); }
            if (Math.abs(timer.milliseconds() - lastSaveMs) >= 25) {
                lastSaveMs = timer.milliseconds();
                pose = followerInstance.getInstance().getPose();
                TimedPoseData pose_data = new TimedPoseData(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()), timer.milliseconds());
                poses.add(pose_data);
            }
        }
        telemetryInstance.addData("Poses:", poses.toArray().length);
        telemetryInstance.addData("Error:", error_str);
        telemetryInstance.addData("Millis:", timer.milliseconds());
        telemetryInstance.update();
    }

    @SuppressLint("SdCardPath")
    public void save() throws IOException {
        stopped = true;
        ObjectMapper mapper = new ObjectMapper();
            mapper.writerWithDefaultPrettyPrinter() // pretty-print
                    .writeValue(new File("/sdcard/FIRST/paths.json"), poses);
    }
}
