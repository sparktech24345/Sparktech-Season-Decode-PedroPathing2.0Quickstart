package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.followerInstance;
import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.telemetryInstance;

import android.annotation.SuppressLint;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.autoRecorder.PoseData;
import org.firstinspires.ftc.teamcode.utils.autoRecorder.PoseSequence;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class AutoRecorder {
    private OpMode opmode;
    private PoseSequence poses = new PoseSequence();
    private double ms = 0;
    private Pose pose = null;
    private boolean always_run = true;
    private BooleanSupplier path_saving;
    private BooleanSupplier pose_adding;
    private double ms_delay = 25;
    private String error_str = "All good!";

    public AutoRecorder(boolean always_run) {
        this.always_run = always_run;
    }

    public AutoRecorder(OpMode opmode) {
        this.opmode = opmode;
    }

    public AutoRecorder(OpMode opmode, boolean always_run) {
        this.opmode = opmode;
        this.always_run = always_run;
    }

    public void setRunMode(boolean always_run) {
        this.always_run = always_run;
    }

    public void setOpMode(OpMode opmode) {
        this.opmode = opmode;
    }

    public String getError() {return error_str;}

    public void setStartMs(double ms) {
        this.ms = ms;
    }

    public void update(double runtime_ms) {
        ms = runtime_ms;
        pose = followerInstance.getInstance().getPose();
        PoseData pose_data = new PoseData(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        poses.addPose(pose_data);
        telemetryInstance.addData("Poses:", poses.poselen());
        telemetryInstance.addData("Error:", error_str);
        telemetryInstance.update();
    }

    @SuppressLint("SdCardPath")
    public void save() throws IOException/*, StreamWriteException, DatabindException*/ {
        ObjectMapper mapper = new ObjectMapper();
            mapper.writerWithDefaultPrettyPrinter() // pretty-print
                    .writeValue(new File("/sdcard/FIRST/paths.json"), poses);
            opmode.requestOpModeStop();
    }
}
