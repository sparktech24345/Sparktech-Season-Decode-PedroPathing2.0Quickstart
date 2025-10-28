package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;


import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.CRServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotState;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;
import java.util.Timer;

@Config
@TeleOp(name="Main TeleOP", group="Main")
public class MainTeleOP extends LinearOpMode {

    private RobotController robot;
    private boolean sorterChoice = false;
    private boolean isInSortingPeriod = false;
    public static double servoP =0;
    public static double servoD =0;
    public boolean isTryingToFire = false;
    public boolean isReadyToFire = false;
    public static  int purpleCounter = 0;
    public static int greenCounter = 0;

    public static int ballCounter = 0;
    private long timerForIntake = 0;

    /// ----------------- Color Sensor Stuff ------------------
    private NormalizedColorSensor colorSensorGreen;
    private NormalizedColorSensor colorSensorPurple;
    private NormalizedRGBA greenSensorColors;
    private NormalizedRGBA purpleSensorColors;


    final float[] hsvValuesGreen = new float[3];
    final float[] hsvValuesPurple = new float[3];
    /// --------------------------------------------------------

    @Override
    public void runOpMode() {
        // init
        ballCounter = 0;

        robot = new RobotController(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), gamepad1, gamepad2) {
            @Override
            public void main_loop() {
                // all of the code

                HandleColors();

                // intakeing

                robot.addTelemetryData("tester",robot.getComponent("IntakeMotor").getPosition());

                if(purpleSensorBall == BallColorSet_Decode.Purple || greenSensorBall == BallColorSet_Decode.Purple || purpleSensorBall == BallColorSet_Decode.Green || greenSensorBall == BallColorSet_Decode.Green){
                    if(ballCounter > 3 && robot.getComponent("IntakeMotor").getPosition() != -1){ //dont infinite stack comands if full reversing already
                        robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL_REVERSE"));
                        robot.addToQueue(new DelayAction(true, 400));
                        robot.addToQueue(new StateAction(true, "IntakeMotor", "OFF"));
                        robot.addTelemetryData("reversing",true);
                        ballCounter--;
                    }
                }
                if(ballCounter >= 3  && timerForIntake + 600 < System.currentTimeMillis()){
                    robot.addToQueue(new StateAction(false, "IntakeSorterServo", "BLOCK"));
                    robot.addTelemetryData("blocking",true);
                }else if(timerForIntake + 600 < System.currentTimeMillis()){
                    robot.addToQueue(new StateAction(false, "IntakeSorterServo", "REDIRECT_TO_PURPLE"));
                    robot.addTelemetryData("redirecting",true);
                }

                if (robot.getControllerKey("A1").IsToggledOnPress) {
                    robot.addToQueue(new StateAction(false, "IntakeMotor", "FULL"));
                    if (
                            (purpleSensorBall == BallColorSet_Decode.Purple
                            || greenSensorBall == BallColorSet_Decode.Purple
                            || purpleSensorBall == BallColorSet_Decode.Green
                            || greenSensorBall == BallColorSet_Decode.Green)
                            && timerForIntake + 1000 < System.currentTimeMillis()
                    ){
                        ballCounter++;
                        timerForIntake = System.currentTimeMillis();
                    }



                    //decide where gate should be opened
                    if (!isInSortingPeriod) {
                        //robot.addToQueue(new StateAction(false, "IntakeSorterServo", "REDIRECT_TO_PURPLE"));
                        robot.addToQueue(new StateAction(false, "GreenGateServo", "CLOSED"));
                        robot.addToQueue(new StateAction(false, "PurpleGateServo", "OPEN"));
                    } else {
                        if(purpleSensorBall == BallColorSet_Decode.Purple || greenSensorBall == BallColorSet_Decode.Purple){
                            purpleCounter++;
                            if(purpleCounter>2){
                                robot.addToQueue(new StateAction(false,"IntakeMotor" ,"SLOW_REVERSE"));
                                purpleCounter--;
                                robot.addToQueue(new DelayAction(true,400));
                                robot.addToQueue(new StateAction(false,"IntakeMotor" ,"OFF"));
                            }
                            sorterChoice = true;
                        }else {
                            greenCounter++;
                            if(greenCounter>1){
                                robot.addToQueue(new StateAction(false , "IntakeMotor","SLOW_REVERSE"));
                                greenCounter--;
                                robot.addToQueue(new DelayAction(true,400));
                                robot.addToQueue(new StateAction(false,"IntakeMotor" ,"OFF"));
                            }
                            sorterChoice = false;
                        }
                    }
                } else {
                    robot.addToQueue(new StateAction(false, "IntakeMotor", "OFF"));
                    robot.addToQueue(new StateAction(false, "IntakeSorterServo", "BLOCK"));
                }

                robot.addTelemetryData("ball counter",ballCounter);


                if (robot.getControllerKey("Y1").ExecuteOnPress) {
                    // robot.togglePowerOff
                }
                if (robot.getControllerKey("RIGHT_BUMPER1").ExecuteOnPress) {
                    setDriveTrainSlowdown(0.2);
                } else setDriveTrainSlowdown(1);

                if (robot.getControllerKey("RIGHT_TRIGGER1").ExecuteOnPress && robot.getControllerKey("LEFT_TRIGGER1").ExecuteOnPress) {
                    setDirectionFlip(!getDirectionFlip());
                }

                if (gamepad1.xWasPressed()) {
                    // shoot
                    robot.addToQueue(new StateAction(true, "TransferServo", "UP"));
                    robot.addToQueue(new DelayAction(true, 600));
                    robot.addToQueue(new StateAction(true, "TransferServo", "DOWN"));
                }

                if (gamepad1.right_bumper) {
                    // output trough intake
                } if (gamepad1.leftBumperWasPressed()){


//
//                    //false is green true is purple
//                    if(sorterChoice){
//                        robot.addToQueue(new StateAction(false, "TurretSpinMotor", "FULL"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "CLOSED"));
//                        robot.addToQueue(new DelayAction(true, 2000));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL"));
//                        robot.addToQueue(new DelayAction(true, 1400));
//                        //robot.addToQueue(new StateAction(true, "PurpleGateServo", "CLOSED"));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "OFF"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "UP"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "DOWN"));
//                        robot.addToQueue(new StateAction(true, "TurretSpinMotor", "OFF"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "OPEN"));
//                    }
//                    else{
//
//                        robot.addToQueue(new StateAction(false, "TurretSpinMotor", "FULL"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "CLOSED"));
//                        robot.addToQueue(new DelayAction(true, 2000));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "FULL"));
//                        robot.addToQueue(new DelayAction(true, 1400));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "CLOSED"));
//                        robot.addToQueue(new StateAction(true, "IntakeMotor", "OFF"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "UP"));
//                        robot.addToQueue(new DelayAction(true, 600));
//                        robot.addToQueue(new StateAction(true, "TransferServo", "DOWN"));
//                        robot.addToQueue(new StateAction(true, "TurretSpinMotor", "OFF"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "OPEN"));
//                    }

                }
                if(gamepad1.rightBumperWasPressed()){
//                    sorterChoice = !sorterChoice;
//
//                    if(sorterChoice){
//                        //is now purple
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "CLOSED"));
//                    }
//                    else{
//                        //is now green
//                        robot.addToQueue(new StateAction(true, "GreenGateServo", "OPEN"));
//                        robot.addToQueue(new StateAction(true, "PurpleGateServo", "CLOSED"));
//                    }
                }



                robot.addTelemetryData("turret power",robot.getCRServoComponent("TurretRotate").getPower());
                robot.addTelemetryData("analog position", robot.getCRServoComponent("TurretRotate").getAnalogPosition());
                robot.addTelemetryData("total analog position",robot.getCRServoComponent("TurretRotate").getServoAnalogTotalPosition());
                robot.addTelemetryData("estimated calculated power",robot.getCRServoComponent("TurretRotate").getCalculatedPower());

                robot.getCRServoComponent("TurretRotate").setPIDconstants(servoP,0,servoD);

                // ================================= DRIVER 2 ===============================================

                if(gamepad2.yWasPressed()) {
                    // Sorting toggle
                    isInSortingPeriod = !isInSortingPeriod;
                }

                if (gamepad2.leftBumperWasPressed()){
                    // odometry false
                }

                if (gamepad2.rightBumperWasPressed()) {
                    // camera targeting false
                }

                if (gamepad2.xWasPressed() || gamepad1.yWasPressed()){
                    isTryingToFire = !isTryingToFire;
                }

                if(gamepad2.aWasPressed()){
                    // take out a ball trough outtake
                }

                if(gamepad2.dpadLeftWasPressed()) {
                    // override portita mov
                    if(robot.getComponent("GreenGateServo").hasStateOfName("CLOSED"))
                        robot.addToQueue(new StateAction(false, "GreenGateServo", "OPEN"));
                    else robot.addToQueue(new StateAction(false, "GreenGateServo", "CLOSED"));
                }

                if(gamepad2.dpadRightWasPressed()) {
                    // override portita verde
                    if(robot.getComponent("PurpleGateServo").hasStateOfName("CLOSED"))
                        robot.addToQueue(new StateAction(false, "PurpleGateServo", "OPEN"));
                    else robot.addToQueue(new StateAction(false, "PurpleGateServo", "CLOSED"));
                }

                if(isTryingToFire){
                    robot.addToQueue(new StateAction(false, "TurretSpinMotor","FULL"));
                    //puterea calculat,unghiul calculat,rotatia calculata;
                }
                else{
                    robot.addToQueue(new StateAction(false, "TurretSpinMotor","OFF"));
                }
            }
        };


        MakeComponents();
        MakeStates();

        colorSensorGreen = hardwareMap.get(NormalizedColorSensor.class, "greensenzor");
        colorSensorPurple = hardwareMap.get(NormalizedColorSensor.class, "purplesenzor");

        robot.UseDefaultMovement();

        while (opModeInInit()) {
            robot.init_loop();
        }

        while(opModeIsActive()) {
            // loop
            robot.loop();
        }
        // stop
    }

    private void MakeComponents() {
        robot.makeComponent("IntakeMotor", new MotorComponent()
                .addMotor("intakemotor")
                .useWithPIDController(false)
                .useWithEncoder(false)
                .setRange(-1, 1)
                .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
        );

        robot.makeComponent("TurretSpinMotor", new MotorComponent()
                .addMotor("turretspin")
                .useWithPIDController(false)
                .useWithEncoder(false)
                .setRange(-1, 1)
        );

        robot.makeComponent("TurretRotate", new CRServoComponent()
                .addMotor("turretrotateleft")
                .setEncoder("leftturretreader")
//                .addMotor("turretrotateright")
                .useWithPIDController(true)
                .setPIDconstants(0, 0, 0)
                .setDirection("turretrotateleft", DcMotorSimple.Direction.REVERSE)
                .setRange(-270,270) // range for PID
                .moveDuringInit(true)
        );

        robot.makeComponent("IntakeSorterServo", new ServoComponent()
                .addMotor("intakeservo")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("PurpleGateServo", new ServoComponent()
                .addMotor("purplegate")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("GreenGateServo", new ServoComponent()
                .addMotor("greengate")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TransferServo", new ServoComponent()
                .addMotor("transferservo")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );

        robot.makeComponent("TurretAngle", new ServoComponent()
                .addMotor("turretangle")
                .setResolution(360)
                .setRange(0, 1)
                .moveDuringInit(true)
        );
    }

    private void MakeStates() {
        robot.getComponent("IntakeMotor")
                .addState("OFF", 0, true)
                .addState("SLOW", 0.5)
                .addState("FULL", 1)
                .addState("FULL_REVERSE", -1)
                .addState("SLOW_REVERSE", -0.5);

        robot.getComponent("TurretSpinMotor")
                .addState("OFF", 0, true)
                .addState("FULL", 1);

        robot.getComponent("PurpleGateServo")
                .addState("OPEN", 35, true)
                .addState("CLOSED", 155);

        robot.getComponent("GreenGateServo")
                .addState("OPEN", 30)
                .addState("CLOSED", 200, true);

        robot.getComponent("IntakeSorterServo")
                .addState("REDIRECT_TO_PURPLE", 161.64)
                .addState("REDIRECT_TO_GREEN", 30)
                .addState("BLOCK", 93.816,true);

        robot.getComponent("TransferServo")
                .addState("DOWN", 30, true)
                .addState("UP", 230);

        robot.getComponent("TurretAngle")
//                .addState("DOWN_MAX", 325, true);
                .addState("DOWN_MAX", 200, true);


        robot.addRobotState("TransferGreen", new RobotState(
                make_pair("GreenGateServo", "OPEN"),
                make_pair("TransferServo", "DOWN"),
                make_pair("IntakeMotor", "FULL")
        ));
        robot.addRobotState("TransferPurple", new RobotState(
                make_pair("PurpleGateServo", "OPEN"),
                make_pair("TransferServo", "DOWN"),
                make_pair("IntakeMotor", "FULL")
        ));
        robot.addRobotState("TransferUp", new RobotState(
                make_pair("IntakeMotor", "OFF"),
                make_pair("PurpleGateServo", "CLOSED"),
                make_pair("GreenGateServo", "CLOSED"),
                make_pair("TransferServo", "UP")
        ));
    }
    private void HandleColors(){
        greenSensorColors =colorSensorGreen.getNormalizedColors();
        purpleSensorColors =colorSensorPurple.getNormalizedColors();

        Color.colorToHSV(greenSensorColors.toColor(), hsvValuesGreen);
        Color.colorToHSV(purpleSensorColors.toColor(), hsvValuesPurple);

        greenSensorBall = BallColorSet_Decode.getColor(greenSensorColors);
        purpleSensorBall = BallColorSet_Decode.getColor(purpleSensorColors);

        /*
        robot.addTelemetryData("G_RED",(double)greenSensorColors.red);
        robot.addTelemetryData("G_BLUE",(double)greenSensorColors.blue);
        robot.addTelemetryData("G_GREEN",(double)greenSensorColors.green);

        robot.addTelemetryData("P_RED",(double)purpleSensorColors.red);
        robot.addTelemetryData("P_BLUE",(double)purpleSensorColors.blue);
        robot.addTelemetryData("P_GREEN",(double)purpleSensorColors.green);
        // */

        robot.addTelemetryData("GREEN_SENSOR_BALL",greenSensorBall);
        robot.addTelemetryData("PURPLE_SENSOR_BALL",purpleSensorBall);





    }
}
