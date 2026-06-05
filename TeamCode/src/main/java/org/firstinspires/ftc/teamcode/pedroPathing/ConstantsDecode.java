package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.control.PredictiveBrakingController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;

import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage;




public class ConstantsDecode {
    public static PredictiveBrakingCoefficients predictiveBrakingCoefficients =
            new PredictiveBrakingCoefficients(0.25,0.075,0.001).withMaximumBrakingPower(0.4);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.3)
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

            .predictiveBrakingCoefficients(predictiveBrakingCoefficients)
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.045, 0.0005))
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

//            .useVoltageCompensation(true) // might be interesting
//            .nominalVoltage(12.8)

            .xVelocity(77.90633)
            .yVelocity(59.8);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.METER)
            .forwardPodY(-0.1119) // -0.1165
            .strafePodX(-0.18467) // -0.155
            .yawScalar(1.000977114)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            100,
            1.6,
            10,
            0.97
    );
    public static PathConstraints pathConstraintsFarAuto = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            100,
            1.5,
            10,
            0.97
    );

    public static Follower createFollowerDecode(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
    public static Follower createFollowerDecodeFarAuto(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraintsFarAuto)
                .build();
    }
}
