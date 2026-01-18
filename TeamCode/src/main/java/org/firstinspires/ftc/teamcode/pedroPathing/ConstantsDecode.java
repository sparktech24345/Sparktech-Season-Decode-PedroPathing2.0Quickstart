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
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyPinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyThreeWheelIMUConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil.SparkyThreeWheelIMULocalizer;

public class ConstantsDecode {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12) // ?????
            .forwardZeroPowerAcceleration(-33.82717174182185)
            .lateralZeroPowerAcceleration(-66.38979558701307)
            .centripetalScaling(0.005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.0001, 0.002, 0.019))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.04, 0.0005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0, 0.6,0));

    public static MecanumConstants mecanumConstants = new MecanumConstants()
//            .leftFrontMotorName(GlobalStorage.frontRightName)
//            .rightFrontMotorName(GlobalStorage.frontLeftName)
//            .leftRearMotorName(GlobalStorage.backRightName)
//            .rightRearMotorName(GlobalStorage.backLeftName)

            .leftFrontMotorName(GlobalStorage.frontLeftName)
            .rightFrontMotorName(GlobalStorage.frontRightName)
            .leftRearMotorName(GlobalStorage.backLeftName)
            .rightRearMotorName(GlobalStorage.backRightName)

            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(76.40856597179503)
            .yVelocity(62.0495009534941);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.METER)
            .forwardPodY(-0.1665)
            .strafePodX(0.015)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

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


    public static ThreeWheelIMUConstants threeWheelIMUConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(6.55511811)
            .rightPodY(-6.55511811)
            .strafePodX(-6.51574803)
            .leftEncoder_HardwareMapName("intakeMotor")
            .rightEncoder_HardwareMapName("frontpurple")
            .strafeEncoder_HardwareMapName("frontgreen")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .customIMU(new CustomIMU() {
                IMU imu;
                int logoFacingDirectionPosition;
                int usbFacingDirectionPosition;
                boolean orientationIsValid = true;
                RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
                        = RevHubOrientationOnRobot.LogoFacingDirection.values();
                RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
                        = RevHubOrientationOnRobot.UsbFacingDirection.values();
                public void updateOrientation(){
                    RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
                    RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
                    try {
                        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
                        imu.initialize(new IMU.Parameters(orientationOnRobot));
                        orientationIsValid = true;
                    } catch (IllegalArgumentException e) {
                        orientationIsValid = false;
                    }
                }
                @Override
                public void initialize(HardwareMap hardwareMap, String hardwareMapName, RevHubOrientationOnRobot hubOrientation) {
                    imu = hardwareMap.get(IMU.class, "imu");
                    logoFacingDirectionPosition = 5; // Right
                    usbFacingDirectionPosition = 0; /// up now
                    updateOrientation();
                }

                @Override
                public double getHeading() {
                    if (orientationIsValid) {
                        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                        return orientation.getYaw(AngleUnit.RADIANS);
                    }
                    else return 0;
                }

                @Override
                public void resetYaw() {
                    imu.resetYaw();
                }
            });
    public static PathConstraints pathConstraints = new PathConstraints( // copiate direct din exemplul Pedro, de verificat / corectat
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1,
            10,
            1
    );

    public static Follower createFollowerDecode(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(pinpointConstants)
                //.threeWheelIMULocalizer(threeWheelIMUConstants)
                //.setLocalizer(new SparkyThreeWheelIMULocalizer(hardwareMap,SparkyLocalizerConstants))
                //.setLocalizer(new SparkyPinpointLocalizer(hardwareMap,sparkyPinpointConstants))
                .build();
    }
}
