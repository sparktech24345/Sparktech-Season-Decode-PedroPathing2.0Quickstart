package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;

public class ConstantsDecode {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(20) // ????
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
            .forwardPodY(6.686102362)
            .strafePodX(0.236240157)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

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

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
