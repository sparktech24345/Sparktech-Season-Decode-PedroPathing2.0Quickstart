/*
 *   Copyright (c) 2025 Alan Smith
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.currentOpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.OpModes;

@TeleOp(name = "GoBilda Pinpoint", group = "Sensor")
public class PinpointTest extends OpMode {
    GoBildaPinpointDriver pinpoint;

    private DcMotor RFDrive;
    private DcMotor LFDrive;
    private DcMotor RBDrive;
    private DcMotor LBDrive;
    MultipleTelemetry tel;
    @Override
    public void init() {
        tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        configurePinpoint();
        if (currentOpModes == OpModes.TeleOP) {
            RFDrive = hardwareMap.get(DcMotor.class, "frontpurple");
            LFDrive = hardwareMap.get(DcMotor.class, "frontgreen");
            RBDrive = hardwareMap.get(DcMotor.class, "backpurple");
            LBDrive = hardwareMap.get(DcMotor.class, "backgreen");

            RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {


        double vertical     = -gamepad1.left_stick_y;
        double horizontal   = -gamepad1.left_stick_x;
        double pivot        = gamepad1.right_stick_x;


        double FrontRightPow = vertical + horizontal - pivot;
        double BackRightPow  = vertical - horizontal - pivot;
        double FrontLeftPow  = vertical - horizontal + pivot;
        double BackLeftPow   = vertical + horizontal + pivot;

        RFDrive.setPower(FrontRightPow);
        LFDrive.setPower(FrontLeftPow);
        RBDrive.setPower(BackRightPow);
        LBDrive.setPower(BackLeftPow);

        telemetry.addLine("Push your robot around to see it track");
        telemetry.addLine("Press A to reset the position");
        if(gamepad1.a){
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        tel.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        tel.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        tel.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

        tel.addData("getDeviceID",pinpoint.getDeviceID());
        tel.addData("getHeading DEGREES",pinpoint.getHeading(AngleUnit.DEGREES));
        tel.addData("UnnormalizedAngleUnit.DEGREES",pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));
        tel.addData("getDeviceID",pinpoint.getDeviceID());
        tel.addData("getLoopTime",pinpoint.getLoopTime());
        tel.addData("getDeviceName",pinpoint.getDeviceName());
        tel.addData("device status",pinpoint.getDeviceStatus());
        tel.addData("getDeviceVersion",pinpoint.getDeviceVersion());
        tel.addData("getHeadingVelocity",pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        tel.addData("getManufacturer",pinpoint.getManufacturer());
        tel.addData("getPosX",pinpoint.getPosX(DistanceUnit.INCH));
        tel.addData("getPosT",pinpoint.getPosY(DistanceUnit.INCH));
        tel.addData("getVelX",pinpoint.getVelX(DistanceUnit.INCH));
        tel.addData("getVelY",pinpoint.getVelY(DistanceUnit.INCH));
        tel.update();
        if(gamepad1.b) pinpoint.recalibrateIMU();
        if(gamepad1.y) pinpoint.resetPosAndIMU();
    }

    public void configurePinpoint(){
        pinpoint.setOffsets(-165.5, -166.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }
}
