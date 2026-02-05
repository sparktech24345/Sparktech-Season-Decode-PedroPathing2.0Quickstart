package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums;

import com.pedropathing.localization.*;

public enum AutoPositions {
    startingPositionDown(0, 0, 0),
    startingPositionUp(0, 0, 0),

    // ball sets
    ballSetUpWall(0, 0, 0),
    ballSetUpCenter(0, 0, 0),



    ballSetMiddleWall(0, 0, 0),
    ballSetMiddleCenter(0, 0, 0),



    ballSetDownWall(0, 0, 0),
    ballSetDownCenter(0, 0, 0),


    ballSetLoadingZoneWall(0, 0, 0),
    ballSetLoadingZoneCenter(0, 0, 0),


    park(0, 0, 0),


    lever(0, 0, 0),


    shootingPointUp(0, 0, 0),
    shootingPointDown(0, 0, 0),

    z(null);

    AutoPositions(Pose pose) {
        this.pose = pose;
    }

    AutoPositions(double x, double y, double headingInDeg) {
        this.pose = new Pose(x, y, Math.toRadians(headingInDeg));
    }

    Pose get() {
        return pose;
    }

    private Pose pose;
}
