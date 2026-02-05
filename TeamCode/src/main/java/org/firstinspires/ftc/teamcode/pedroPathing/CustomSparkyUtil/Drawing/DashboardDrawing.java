package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.Drawing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.DashboardPoseTracker;
//
///**
// * This is the Drawing class. It handles the drawing of stuff on FTC Dashboard, like the robot.
// *
// * @author Logan Nash
// * @author Anyi Lin - 10158 Scott's Bots
// * @version 1.0, 4/22/2024
// */
//public class DashboardDrawing {
//    public static final double ROBOT_RADIUS = 9;
//
//    private static TelemetryPacket packet;
//    private final Follower follower;
//    private final DashboardPoseTracker dashboardPoseTracker;
//    public DashboardDrawing(Follower follower, DashboardPoseTracker dashboardPoseTracker){
//        this.follower = follower;
//        this.dashboardPoseTracker = dashboardPoseTracker;
//    }
//
//
//    public void update(){
//        dashboardPoseTracker.update();
//        drawDebug();
//    }
//
//    public void drawDebug() {
//        if (follower.getCurrentPath() != null) {
//            drawPath(follower.getCurrentPath(), "#3F51B5");
//            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
//            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), "#3F51B5");
//        }
//        drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//        drawRobot(follower.getPose(), "#4CAF50");
//
//        sendPacket();
//    }
//
//    public static void drawRobot(Pose pose, String color) {
//        if (packet == null) packet = new TelemetryPacket();
//
//        packet.fieldOverlay().setStroke(color);
//        DashboardDrawing.drawRobotOnCanvas(packet.fieldOverlay(), pose.copy());
//    }
//
//    public static void drawRobot(Pose pose){drawRobot(pose,"#4CAF50"); }
//    public static void drawPath(Path path, String color) {
//        if (packet == null) packet = new TelemetryPacket();
//
//        packet.fieldOverlay().setStroke(color);
//        DashboardDrawing.drawPath(packet.fieldOverlay(), path.getPanelsDrawingPoints());
//    }
//
//    public static void drawPath(PathChain pathChain, String color) {
//        for (int i = 0; i < pathChain.size(); i++) {
//            drawPath(pathChain.getPath(i), color);
//        }
//    }
//
//    public static void drawPoseHistory(DashboardPoseTracker poseTracker, String color) {
//        if (packet == null) packet = new TelemetryPacket();
//
//        packet.fieldOverlay().setStroke(color);
//        packet.fieldOverlay().strokePolyline(poseTracker.getXPositionsArray(), poseTracker.getYPositionsArray());
//    }
//
//    public static boolean sendPacket() {
//        if (packet != null) {
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//            packet = null;
//            return true;
//        }
//        return false;
//    }
//
//    public static void drawRobotOnCanvas(Canvas c, Pose t) {
//        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);
//        Vector v = t.getHeadingAsUnitVector();
//        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
//        double x1 = t.getX() + v.getXComponent() / 2, y1 = t.getY() + v.getYComponent() / 2;
//        double x2 = t.getX() + v.getXComponent(), y2 = t.getY() + v.getYComponent();
//        c.strokeLine(x1, y1, x2, y2);
//    }
//
//    public static void drawPath(Canvas c, double[][] points) {
//        c.strokePolyline(points[0], points[1]);
//    }
//}
