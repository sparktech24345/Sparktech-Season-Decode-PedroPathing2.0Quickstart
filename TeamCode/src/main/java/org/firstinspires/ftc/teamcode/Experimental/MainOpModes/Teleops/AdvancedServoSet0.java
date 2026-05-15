package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@TeleOp(name="AdvServo0", group="Debug")
public class AdvancedServoSet0 extends ComplexOpMode {

    public HashMap<String, ServoImplEx> servos = new HashMap<>(4);

    @Override
    public void initialize() {

    }

    @Override
    public void update() {

    }

    @Override
    public void telemetry() {

    }
}
