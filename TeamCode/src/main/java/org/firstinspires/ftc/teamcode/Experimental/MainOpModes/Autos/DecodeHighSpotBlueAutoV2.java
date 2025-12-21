package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Config
@Disabled
@Autonomous(name = "Auto big triangle blue", group = "Tests")
public class DecodeHighSpotBlueAutoV2 extends DecodeLowTriangleBlueAuto {
    @Override
    public Pose getStartingPose() {
        return classifier_starter;
    }

    @Override
    public void setShootState() {
        autoShootState = AutoEnumStates.ShootingFromBigTriangle;
    }
}
