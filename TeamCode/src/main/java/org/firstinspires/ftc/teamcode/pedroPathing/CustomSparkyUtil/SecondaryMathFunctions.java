////
//// Source code recreated from a .class file by IntelliJ IDEA
//// (powered by FernFlower decompiler)
////
//
//package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;
//
//import com.pedropathing.localization.Pose;
//import com.pedropathing.math.Vector;
//
//public class SecondaryMathFunctions {
//    public SecondaryMathFunctions() {
//    }
//
//    public static double nCr(int n, int r) {
//        double num = 1.0;
//        double denom = 1.0;
//
//        int i;
//        for(i = n; i > n - r; --i) {
//            num *= (double)i;
//        }
//
//        for(i = 1; i <= r; ++i) {
//            denom *= (double)i;
//        }
//
//        return num / denom;
//    }
//
//    public static double getSign(double get) {
//        if (get == 0.0) {
//            return 0.0;
//        } else {
//            return get > 0.0 ? 1.0 : -1.0;
//        }
//    }
//
//    public static double clamp(double num, double lower, double upper) {
//        if (num < lower) {
//            return lower;
//        } else {
//            return num > upper ? upper : num;
//        }
//    }
//
//    public static double normalizeAngle(double angleRadians) {
//        double angle = angleRadians % 6.283185307179586;
//        return angle < 0.0 ? angle + 6.283185307179586 : angle;
//    }
//
//    public static double getSmallestAngleDifference(double one, double two) {
//        return Math.min(normalizeAngle(one - two), normalizeAngle(two - one));
//    }
//
//    public static double getTurnDirection(double startHeading, double endHeading) {
//        return normalizeAngle(endHeading - startHeading) >= 0.0 && normalizeAngle(endHeading - startHeading) <= Math.PI ? 1.0 : -1.0;
//    }
//
//    public static double distance(Pose one, Pose two) {
//        return Math.sqrt(Math.pow(one.getX() - two.getX(), 2.0) + Math.pow(one.getY() - two.getY(), 2.0));
//    }
//
//    public static Pose addPoses(Pose one, Pose two) {
//        return new Pose(one.getX() + two.getX(), one.getY() + two.getY(), one.getHeading() + two.getHeading());
//    }
//
//    public static Pose subtractPoses(Pose one, Pose two) {
//        return new Pose(one.getX() - two.getX(), one.getY() - two.getY(), one.getHeading() - two.getHeading());
//    }
//
//    public static Pose rotatePose(Pose pose, double theta, boolean rotateHeading) {
//        double x = pose.getX() * Math.cos(theta) - pose.getY() * Math.sin(theta);
//        double y = pose.getX() * Math.sin(theta) + pose.getY() * Math.cos(theta);
//        double heading = rotateHeading ? normalizeAngle(pose.getHeading() + theta) : pose.getHeading();
//        return new Pose(x, y, heading);
//    }
//
//
//    public static Vector copyVector(Vector vector) {
//        return new Vector(vector.getMagnitude(), vector.getTheta());
//    }
//
//    public static Vector scalarMultiplyVector(Vector vector, double scalar) {
//        return new Vector(vector.getMagnitude() * scalar, vector.getTheta());
//    }
//
//    public static Vector normalizeVector(Vector vector) {
//        return vector.getMagnitude() == 0.0 ? new Vector(0.0, vector.getTheta()) : new Vector(vector.getMagnitude() / Math.abs(vector.getMagnitude()), vector.getTheta());
//    }
//
//    public static Vector addVectors(Vector one, Vector two) {
//        Vector returnVector = new Vector();
//        returnVector.setOrthogonalComponents(one.getXComponent() + two.getXComponent(), one.getYComponent() + two.getYComponent());
//        return returnVector;
//    }
//
//    public static Vector subtractVectors(Vector one, Vector two) {
//        Vector returnVector = new Vector();
//        returnVector.setOrthogonalComponents(one.getXComponent() - two.getXComponent(), one.getYComponent() - two.getYComponent());
//        return returnVector;
//    }
//
//    public static double dotProduct(Vector one, Vector two) {
//        return one.getXComponent() * two.getXComponent() + one.getYComponent() * two.getYComponent();
//    }
//
//    public static double crossProduct(Vector one, Vector two) {
//        return one.getXComponent() * two.getYComponent() - one.getYComponent() * two.getXComponent();
//    }
//
//    public static boolean roughlyEquals(double one, double two, double accuracy) {
//        return one < two + accuracy && one > two - accuracy;
//    }
//
//    public static boolean roughlyEquals(double one, double two) {
//        return roughlyEquals(one, two, 1.0E-4);
//    }
//
//    public static double inToMM(double in) {
//        return in * 25.4;
//    }
//
//    public static double mmToIn(double mm) {
//        return mm / 25.4;
//    }
//}
