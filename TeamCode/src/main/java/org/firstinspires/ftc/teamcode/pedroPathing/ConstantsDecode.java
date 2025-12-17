package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyPinpointConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyPinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyThreeWheelIMUConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyThreeWheelIMULocalizer;

public class ConstantsDecode {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14) // ?????
            .forwardZeroPowerAcceleration(-34.62719) // copiate direct din exemplul Pedro, de verificat / corectat
            .lateralZeroPowerAcceleration(-78.15554) // copiate direct din exemplul Pedro, de verificat / corectat
            .centripetalScaling(0.0005) // copiate direct din exemplul Pedro, de verificat / corectat
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.004, 0))//(0.67, 0, 0.45, 0.023))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.04, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0, 0.006, 0.6,0));

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName(GlobalStorage.frontRightName)
            .rightFrontMotorName(GlobalStorage.frontLeftName)
            .leftRearMotorName(GlobalStorage.backRightName)
            .rightRearMotorName(GlobalStorage.backLeftName)

            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(78.261926752421046666666666666667) // copiate direct din exemplul Pedro, de verificat / corectat
            .yVelocity(61.494551922189565); // copiate direct din exemplul Pedro, de verificat / corectat

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.METER) //might be changed later
            .forwardPodY(-0.177563) //-0.172 // -0.177563
            .strafePodX(0.09647796) //0.006 //0.09647796
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static SparkyPinpointConstants sparkyPinpointConstants = new SparkyPinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.METER) //might be changed later
            .forwardPodY(-0.177563) //-0.172 // -0.177563
            .strafePodX(0.09647796) //0.006 //0.09647796
            .forwardEncoderDirection(SparkyPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(SparkyPinpointDriver.EncoderDirection.FORWARD);

    public static SparkyThreeWheelIMUConstants SparkyLocalizerConstants = new SparkyThreeWheelIMUConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(6.299213)
            .rightPodY(-6.299213)
            .strafePodX(-0.09647796)
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
                //.setLocalizer(new SparkyPinpointLocalizer(hardwareMap,sparkyPinpointConstants))
                .build();
    }
}
