//package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos.tuners_tests.examples;
//
//import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexFollower.improviseBezierCurvePath;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.FConstantsForPinpoint;
//import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.LConstantsForPinpoint;
//
//
///**
// * This is the Triangle autonomous OpMode.
// * It runs the robot in a triangle, with the starting point being the bottom-middle point.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
// * @version 1.0, 12/30/2024
// */
//@Autonomous(name = "Triangle", group = "Examples")
//
//public class Triangle extends OpMode {
//    private Follower follower;
//
//    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
//    private final Pose interPose = new Pose(24, -24, Math.toRadians(-90));
//    private final Pose endPose = new Pose(24, 24, Math.toRadians(0));
//
//    private PathChain triangle;
//
//    private Telemetry telemetryA;
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the FTC Dashboard.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//
//        if (follower.atParametricEnd()) {
//            follower.followPath(triangle, false);
//        }
//
//        follower.telemetryDebug(telemetryA);
//    }
//
//    /**
//     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
//     * initializes the FTC Dashboard telemetry.
//     */
//    @Override
//    public void init() {
//        Constants.setConstants(FConstantsForPinpoint.class, LConstantsForPinpoint.class);
//        follower = new Follower(hardwareMap,FConstantsForPinpoint.class,LConstantsForPinpoint.class);
//        follower.setStartingPose(startPose);
//
////        triangle = follower.pathBuilder()
////                .addPath(new BezierLine(new Point(startPose), new Point(interPose)))
////                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
////                .addPath(new BezierLine(new Point(interPose), new Point(endPose)))
////                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
////                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
////                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
////                .build();
//
//        Path curve1 = improviseBezierCurvePath(startPose,interPose,true);
//        Path curve2 = improviseBezierCurvePath(interPose,endPose,false);
//        curve2.setReversed(true);
//        Path curve3 = improviseBezierCurvePath(endPose,startPose,true);
//        curve3.setReversed(true);
//
//        triangle = follower.pathBuilder()
//                .addPath(curve1)
//                .addPath(new BezierLine(new Point(interPose), new Point(endPose)))
//                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
//                .addPath(curve3)
//                .build();
//
//        follower.followPath(triangle);
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("This will run in a roughly triangular shape,"
//                + "starting on the bottom-middle point. So, make sure you have enough "
//                + "space to the left, front, and right to run the OpMode.");
//        telemetryA.update();
//    }
//
//}
