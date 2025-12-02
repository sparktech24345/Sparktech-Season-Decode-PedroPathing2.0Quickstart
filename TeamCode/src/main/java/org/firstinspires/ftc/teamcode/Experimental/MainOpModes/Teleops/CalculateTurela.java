package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.targetX;
import static org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops.MainTeleOP.targetY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.TrajectoryCalculator;

@TeleOp(name = "CalculateTurela", group = "Tests")
public class CalculateTurela extends LinearOpMode {
    public RobotController robot;
    private TrajectoryCalculator trajectoryCalculator = new TrajectoryCalculator();

    // Constants
    private static final double GRAVITY = 9.80665;
    private static final double LAUNCH_VELOCITY = 12.0; // m/s (example – measure your shooter)
    private static final double SHOOTER_HEIGHT = 0.38; // meters
    private static final double BASKET_HEIGHT = 1.2; // meters

    @Override
    public void runOpMode() {
        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                double distance = trajectoryCalculator.calculateDistance(robot.getCurrentPose(), new Pose(targetX, targetY, 0), true);

                double angleDeg = calculateLaunchAngle(distance,
                        LAUNCH_VELOCITY,
                        BASKET_HEIGHT - SHOOTER_HEIGHT);

                robot.addTelemetryData("angle: ", angleDeg);
            }
        };

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            robot.loop();
        }
    }

    /**
     * Calculates the vertical launch angle needed to hit a target.
     *
     * @param distance horizontal distance to target in meters
     * @param velocity launch velocity in m/s
     * @param heightDifference targetHeight - shooterHeight (meters)
     * @return angle in degrees
     */
    public double calculateLaunchAngle(double distance, double velocity, double heightDifference) {

        double g = GRAVITY;
        double v2 = velocity * velocity;

        // Expression inside the square root
        double discriminant = v2 * v2 - g * (g * distance * distance + 2 * heightDifference * v2);

        if (discriminant < 0) {
            // No real solution → shooter too weak
            return Double.NaN;
        }

        double sqrtTerm = Math.sqrt(discriminant);

        // Lower angle trajectory
        double angleRad = Math.atan((v2 - sqrtTerm) / (g * distance));

        return Math.toDegrees(angleRad);
    }

}
