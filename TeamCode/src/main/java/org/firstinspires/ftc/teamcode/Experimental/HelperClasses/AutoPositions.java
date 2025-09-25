package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.pedropathing.geometry.*;
public enum AutoPositions {
    // starting positions
    startingPositionDown(new Pose(0,0,0)),
    startingPositionUp(new Pose(0,0,0)),



    // ball sets
    ballSetUpWall(new Pose(0,0,0)),
    ballSetUpCenter(new Pose(0,0,0)),

    // ---------------------------------------------- \\

    ballSetMiddleWall(new Pose(0,0,0)),
    ballSetMiddleCenter(new Pose(0,0,0)),

    // ---------------------------------------------- \\

    ballSetDownWall(new Pose(0,0,0)),
    ballSetDownCenter(new Pose(0,0,0)),

    // ---------------------------------------------- \\

    ballSetLoadingZoneWall(new Pose(0,0,0)),
    ballSetLoadingZoneCenter(new Pose(0,0,0)),



    // parks
    park(new Pose(0,0,0)),


    // lever
    lever(new Pose(0,0,0)),

    //shooting zones
    shootingPointUp(new Pose(0,0,0)),
    shootingPointDown(new Pose(0,0,0));

    AutoPositions(Pose pose){
    }
    private Pose pose;
}
