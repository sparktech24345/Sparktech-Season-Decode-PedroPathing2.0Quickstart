package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyThreeWheelIMUConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyThreeWheelIMULocalizer;

public class ConstantsDecode {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.5) // ????
            .forwardZeroPowerAcceleration(-25.9346931313679598) // copiate direct din exemplul Pedro, de verificat / corectat
            .lateralZeroPowerAcceleration(-67.342491844080064) // copiate direct din exemplul Pedro, de verificat / corectat
            .centripetalScaling(0.0005); // copiate direct din exemplul Pedro, de verificat / corectat

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName(GlobalStorage.frontLeftName)
            .rightFrontMotorName(GlobalStorage.frontRightName)
            .leftRearMotorName(GlobalStorage.backLeftName)
            .rightRearMotorName(GlobalStorage.backRightName)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(78.261926752421046666666666666667) // copiate direct din exemplul Pedro, de verificat / corectat
            .yVelocity(61.494551922189565); // copiate direct din exemplul Pedro, de verificat / corectat

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.METER) //might be changed later
            .forwardPodY(-0.172)
            .strafePodX(-0.006)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static SparkyThreeWheelIMUConstants SparkyLocalizerConstants = new SparkyThreeWheelIMUConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(6.76484252)
            .rightPodY(-6.76484252)
            .strafePodX(0.23622)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("frontpurple")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .setPinpoint_HardwareName("pinpoint");
    public static PathConstraints pathConstraints = new PathConstraints( // copiate direct din exemplul Pedro, de verificat / corectat
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );

    public static Follower createFollowerDecode(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(pinpointConstants)
                //.setLocalizer(new SparkyThreeWheelIMULocalizer(hardwareMap,SparkyLocalizerConstants))
                .build();
    }
}
