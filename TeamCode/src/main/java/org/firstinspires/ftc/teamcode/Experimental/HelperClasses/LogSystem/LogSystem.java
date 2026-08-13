package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.LogSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import android.annotation.SuppressLint;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;
@Config
public class LogSystem {

    protected RobotController robot;
    private boolean errors = false;
    private PrintWriter logWriter;
    protected VoltageSensor controlHubVoltageSensor;

    private String gamepadLog = "KeyPressed:";

    public void updateLoop() {
        //this needs to happen every loop
        writeLog("NEW_CYCLE:"+System.currentTimeMillis());
        checkForGamepadInputD1();
        checkForGamepadInputD2();
        checkForMotors();
        checkForServos();
        checkForVoltage();
        checkForPosition();
    }

    /**
     * Call this ONCE at startup (e.g., in init() or at the start of your OpMode)
     */
    @SuppressLint("SdCardPath")
    public void createFile() {
        try {
            File logDir = new File("/sdcard/FIRST/logs");
            if (!logDir.exists()) {
                logDir.mkdirs(); // Creates missing parent directories safely
            }

            SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US);
            formatter.setTimeZone(TimeZone.getTimeZone("Europe/Bucharest"));
            String fileName = "log_" + formatter.format(new Date()) + ".txt";

            // Wrap FileWriter in BufferedWriter and PrintWriter for fast I/O performance
            FileWriter fw = new FileWriter(fileName, true);
            BufferedWriter bw = new BufferedWriter(fw);
            logWriter = new PrintWriter(bw);

            logWriter.println("=== Log Started ===");
        } catch (IOException e) {
            errors = true;
        }

    }

    /**
     * Call this anywhere in your code, even inside a 20ms loop!
     */
    public void writeLog(String message) {
        if (logWriter != null) {
            logWriter.println(message);
        }
    }

    /**
     * Call this ONCE when finished (e.g., in stop() or at the end of the OpMode)
     */

    public void closeFile() {
        if (logWriter != null) {
            logWriter.flush(); // Ensures all buffered data is saved to disk
            logWriter.close();
            logWriter = null;
        }
    }

    //DRIVER INPUT FUNCTIONS

    public void checkForGamepadInputD1() {
        if(gamepad1.aWasPressed()) writeLog(gamepadLog + "A1:" + gamepad1.a);
        if(gamepad1.bWasPressed()) writeLog(gamepadLog + "B1:" + gamepad1.b);
        if(gamepad1.xWasPressed()) writeLog(gamepadLog + "X1:" + gamepad1.x);
        if(gamepad1.yWasPressed()) writeLog(gamepadLog + "Y1:" + gamepad1.y);

        if(gamepad1.dpadUpWasPressed()) writeLog(gamepadLog + "DPAD_UP1:" + gamepad1.dpad_up);
        if(gamepad1.dpadDownWasPressed()) writeLog(gamepadLog + "DPAD_DOWN1:" + gamepad1.dpad_down);
        if(gamepad1.dpadRightWasPressed()) writeLog(gamepadLog + "DPAD_RIGHT1:" + gamepad1.dpad_right);
        if(gamepad1.dpadLeftWasPressed()) writeLog(gamepadLog + "DPAD_LEFT1:" + gamepad1.dpad_left);

        if(gamepad1.startWasPressed()) writeLog(gamepadLog + "START1:" + gamepad1.start);
        if(gamepad1.backWasPressed()) writeLog(gamepadLog + "BACK1:" + gamepad1.back);

        if(gamepad1.leftBumperWasPressed()) writeLog(gamepadLog + "LEFT_BUMPER1:" + gamepad1.left_bumper);
        if(gamepad1.rightBumperWasPressed()) writeLog(gamepadLog + "RIGHT_BUMPER1:" + gamepad1.right_bumper);

        if(gamepad1.leftStickButtonWasPressed()) writeLog(gamepadLog + "LEFT_STICK_BUTTON1:" + gamepad1.left_stick_button);
        if(gamepad1.rightStickButtonWasPressed()) writeLog(gamepadLog + "RIGHT_STICK_BUTTON1:" + gamepad1.right_stick_button);

        if(gamepad1.optionsWasPressed()) writeLog(gamepadLog + "OPTIONS1:" + gamepad1.options);

        writeLog("STICK:LEFT_STICK_X1:" + gamepad1.left_stick_x);
        writeLog("STICK:LEFT_STICK_Y1:" + gamepad1.left_stick_y);

        writeLog("STICK:RIGHT_STICK_X1:" + gamepad1.right_stick_x);
        writeLog("STICK:RIGHT_STICK_Y1:" + gamepad1.right_stick_y);

        writeLog("TRIGGER:LEFT_TRIGGER1" + gamepad1.left_trigger);
        writeLog("TRIGGER:RIGHT_TRIGGER1:" + gamepad1.right_trigger);
    }

    public void checkForGamepadInputD2() {
        if(gamepad2.aWasPressed()) writeLog(gamepadLog + "A2:" + gamepad2.a);
        if(gamepad2.bWasPressed()) writeLog(gamepadLog + "B2:" + gamepad2.b);
        if(gamepad2.xWasPressed()) writeLog(gamepadLog + "X2:" + gamepad2.x);
        if(gamepad2.yWasPressed()) writeLog(gamepadLog + "Y2:" + gamepad2.y);

        if(gamepad2.dpadUpWasPressed()) writeLog(gamepadLog + "DPAD_UP2:" + gamepad2.dpad_up);
        if(gamepad2.dpadDownWasPressed()) writeLog(gamepadLog + "DPAD_DOWN2:" + gamepad2.dpad_down);
        if(gamepad2.dpadRightWasPressed()) writeLog(gamepadLog + "DPAD_RIGHT2:" + gamepad2.dpad_right);
        if(gamepad2.dpadLeftWasPressed()) writeLog(gamepadLog + "DPAD_LEFT2:" + gamepad2.dpad_left);

        if(gamepad2.startWasPressed()) writeLog(gamepadLog + "START2:" + gamepad2.start);
        if(gamepad2.backWasPressed()) writeLog(gamepadLog + "BACK2:" + gamepad2.back);

        if(gamepad2.leftBumperWasPressed()) writeLog(gamepadLog + "LEFT_BUMPER2:" + gamepad2.left_bumper);
        if(gamepad2.rightBumperWasPressed()) writeLog(gamepadLog + "RIGHT_BUMPER2:" + gamepad2.right_bumper);

        if(gamepad2.leftStickButtonWasPressed()) writeLog(gamepadLog + "LEFT_STICK_BUTTON2:" + gamepad2.left_stick_button);
        if(gamepad2.rightStickButtonWasPressed()) writeLog(gamepadLog + "RIGHT_STICK_BUTTON2:" + gamepad2.right_stick_button);

        if(gamepad2.optionsWasPressed()) writeLog(gamepadLog + "OPTIONS2:" + gamepad2.options);

        writeLog("STICK:LEFT_STICK_X2:" + gamepad2.left_stick_x);
        writeLog("STICK:LEFT_STICK_Y2:" + gamepad2.left_stick_y);

        writeLog("STICK:RIGHT_STICK_X2:" + gamepad2.right_stick_x);
        writeLog("STICK:RIGHT_STICK_Y2:" + gamepad2.right_stick_y);

        writeLog("TRIGGER:LEFT_TRIGGER2:" + gamepad2.left_trigger);
        writeLog("TRIGGER:RIGHT_TRIGGER2:" + gamepad2.right_trigger);
    }


    // MOTOR DATA FUNCTION
    public void checkForMotors() {
        for(MotorNames motorName : MotorNames.values()) {
            writeLog("MOTOR:" + motorName + ":POWER" + robot.getMotorComponent(String.valueOf(motorName)).getPower());
            writeLog("MOTOR:" + motorName + ":VELOCITY" + robot.getMotorComponent(String.valueOf(motorName)).getVelocity());
            writeLog("MOTOR:" + motorName + ":CURRENT" + robot.getMotorComponent(String.valueOf(motorName)).getCurrent());
            writeLog("MOTOR:" + motorName + ":POSITION" + robot.getMotorComponent(String.valueOf(motorName)).getPosition());
            writeLog("MOTOR:" + motorName + ":ABS_POSITION" + robot.getMotorComponent(String.valueOf(motorName)).getAbsolutePosition());
        }
    }

    public void checkForServos() {
        for(ServoNames servoName : ServoNames.values()) {
            writeLog("SERVO:" + servoName + ":POSITION" + robot.getServoComponent(String.valueOf(servoName)).getPosition());
        }
    }

    public void checkForPosition() {
        writeLog("ROBOT_POSITION:" + robot.getCurrentPose());
    }

    public void checkForVoltage() {
        writeLog("GENERAL_VOLTAGE:" + controlHubVoltageSensor.getVoltage());
    }
}
