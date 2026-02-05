//
//
package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;
//
//public class DashboardPoseTracker {
//    private double[] xPositions;
//    private double[] yPositions;
//    private PoseUpdater poseUpdater;
//    private long lastUpdateTime;
//    private final int TRACKING_LENGTH = 1500;
//    private final long UPDATE_TIME = 50L;
//    private final int TRACKING_SIZE = 30;
//
//    public DashboardPoseTracker(PoseUpdater poseUpdater) {
//        this.poseUpdater = poseUpdater;
//        this.xPositions = new double[30];
//        this.yPositions = new double[30];
//
//        for(int i = 0; i < 30; ++i) {
//            this.xPositions[i] = poseUpdater.getPose().getX();
//            this.yPositions[i] = poseUpdater.getPose().getY();
//        }
//
//        this.lastUpdateTime = System.currentTimeMillis() - 50L;
//    }
//
//    public void update() {
//        if (System.currentTimeMillis() - this.lastUpdateTime > 50L) {
//            this.lastUpdateTime = System.currentTimeMillis();
//
//            for(int i = 29; i > 0; --i) {
//                this.xPositions[i] = this.xPositions[i - 1];
//                this.yPositions[i] = this.yPositions[i - 1];
//            }
//
//            this.xPositions[0] = this.poseUpdater.getPose().getX();
//            this.yPositions[0] = this.poseUpdater.getPose().getY();
//        }
//
//    }
//
//    public double[] getXPositionsArray() {
//        return this.xPositions;
//    }
//
//    public double[] getYPositionsArray() {
//        return this.yPositions;
//    }
//}
