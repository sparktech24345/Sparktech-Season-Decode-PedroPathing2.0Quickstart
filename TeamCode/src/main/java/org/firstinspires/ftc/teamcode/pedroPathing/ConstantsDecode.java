package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyPinpointConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyPinpointDriver;

import java.nio.file.Path;

public class ConstantsDecode {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-27.81051833)
            .lateralZeroPowerAcceleration(-55.7462)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.004, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.045, 0.0005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03,0,0.0037,0.6,0.02))

            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0002, 0.6,0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.057,0,0.004,0.02))


            .useSecondaryDrivePIDF(false)
            .useSecondaryTranslationalPIDF(false)
            ;

    public static MecanumConstants mecanumConstants = new MecanumConstants()

            .leftFrontMotorName(GlobalStorage.frontLeftName)
            .rightFrontMotorName(GlobalStorage.frontRightName)
            .leftRearMotorName(GlobalStorage.backLeftName)
            .rightRearMotorName(GlobalStorage.backRightName)

            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(77.90633)
            .yVelocity(59.8);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.METER)
            .forwardPodY(-0.1165)
            .strafePodX(-0.11938)
            .yawScalar(1.001168)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            100,
            1.5,
            10,
            1
    );


    public static Follower createFollowerDecode(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
