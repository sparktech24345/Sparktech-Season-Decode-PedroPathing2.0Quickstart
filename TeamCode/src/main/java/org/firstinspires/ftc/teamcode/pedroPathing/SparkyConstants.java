package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyTwoWheelLocalizer;

public class SparkyConstants {

    public static FollowerConstants ITDfollowerConstants = new FollowerConstants()
            .mass(11.8)
            .forwardZeroPowerAcceleration(-45)
            .lateralZeroPowerAcceleration(-95)
            .centripetalScaling(0.0005);

    public static MecanumConstants ITDdriveConstants = new MecanumConstants()
            .leftFrontMotorName("frontleft")
            .leftRearMotorName("backleft")
            .rightFrontMotorName("frontright")
            .rightRearMotorName("backright")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(0.6 * 78)
            .yVelocity(0.6 * 63);

    public static PathConstraints ITDpathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            200,
            5,
            10,
            1
    );

    public static Follower SparkycreateFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(ITDfollowerConstants, hardwareMap)
                .mecanumDrivetrain(ITDdriveConstants)
                .setLocalizer(new SparkyTwoWheelLocalizer(hardwareMap,new TwoWheelConstants(),new Pose(0,0,0)))
                .pathConstraints(ITDpathConstraints)
                .build();
    }
}