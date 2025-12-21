package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
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
