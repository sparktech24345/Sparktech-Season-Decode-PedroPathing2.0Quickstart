package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.LogSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.MotorComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.ServoComponent;
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

    private static final String gamepadLog = "KeyPressed:";

    // Buffer size for the underlying BufferedWriter - big enough to absorb
    // many loop cycles before the OS actually has to touch disk.
    private static final int WRITER_BUFFER_BYTES = 262144; // 256KB

    // One StringBuilder reused every cycle instead of allocating small
    // strings + doing ~30 println() calls per loop.
    private final StringBuilder cycleBuffer = new StringBuilder(4096);

    public void updateLoop() {
        cycleBuffer.setLength(0); // reset, no new allocation

        cycleBuffer.append("NEW_CYCLE:").append(System.currentTimeMillis()).append('\n');

        checkForGamepadInputD1();
        checkForGamepadInputD2();
        checkForMotors();
        checkForServos();
        checkForVoltage();
        checkForPosition();

        // Single write call per loop instead of ~30
        flushCycleToWriter();
    }

    private void flushCycleToWriter() {
        if (logWriter != null) {
            logWriter.print(cycleBuffer);
            // Deliberately NOT calling flush() here - let the BufferedWriter's
            // internal buffer absorb it. It only touches disk when that
            // buffer (WRITER_BUFFER_BYTES) fills up.
        }
    }

    /**
     * Call this ONCE at startup (e.g., in init() or at the start of your OpMode)
     */
    @SuppressLint("SdCardPath")
    public void createFile() {
        try {
            File logDir = new File("/sdcard/FIRST/logs");
            if (!logDir.exists()) {
                logDir.mkdirs();
            }

            SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US);
            formatter.setTimeZone(TimeZone.getTimeZone("Europe/Bucharest"));
            String fileName = "log_" + formatter.format(new Date()) + ".txt";

            FileWriter fw = new FileWriter(new File(logDir, fileName), true);
            BufferedWriter bw = new BufferedWriter(fw, WRITER_BUFFER_BYTES);
            logWriter = new PrintWriter(bw, false); // autoFlush = false, critical

            logWriter.println("=== Log Started ===");
        } catch (IOException e) {
            errors = true;
        }
    }

    /**
     * Appends a line to this cycle's buffer instead of writing immediately.
     * Safe to call many times per loop - it's just StringBuilder.append().
     */
    public void writeLog(String message) {
        cycleBuffer.append(message).append('\n');
    }

    /**
     * Call this ONCE when finished (e.g., in stop() or at the end of the OpMode)
     */
    public void closeFile() {
        if (logWriter != null) {
            logWriter.flush();
            logWriter.close();
            logWriter = null;
        }
    }

    // DRIVER INPUT FUNCTIONS

    public void checkForGamepadInputD1() {
        if (gamepad1.aWasPressed()) writeLog(gamepadLog + "A1:" + gamepad1.a);
        if (gamepad1.bWasPressed()) writeLog(gamepadLog + "B1:" + gamepad1.b);
        if (gamepad1.xWasPressed()) writeLog(gamepadLog + "X1:" + gamepad1.x);
        if (gamepad1.yWasPressed()) writeLog(gamepadLog + "Y1:" + gamepad1.y);

        if (gamepad1.dpadUpWasPressed()) writeLog(gamepadLog + "DPAD_UP1:" + gamepad1.dpad_up);
        if (gamepad1.dpadDownWasPressed()) writeLog(gamepadLog + "DPAD_DOWN1:" + gamepad1.dpad_down);
        if (gamepad1.dpadRightWasPressed()) writeLog(gamepadLog + "DPAD_RIGHT1:" + gamepad1.dpad_right);
        if (gamepad1.dpadLeftWasPressed()) writeLog(gamepadLog + "DPAD_LEFT1:" + gamepad1.dpad_left);

        if (gamepad1.startWasPressed()) writeLog(gamepadLog + "START1:" + gamepad1.start);
        if (gamepad1.backWasPressed()) writeLog(gamepadLog + "BACK1:" + gamepad1.back);

        if (gamepad1.leftBumperWasPressed()) writeLog(gamepadLog + "LEFT_BUMPER1:" + gamepad1.left_bumper);
        if (gamepad1.rightBumperWasPressed()) writeLog(gamepadLog + "RIGHT_BUMPER1:" + gamepad1.right_bumper);

        if (gamepad1.leftStickButtonWasPressed()) writeLog(gamepadLog + "LEFT_STICK_BUTTON1:" + gamepad1.left_stick_button);
        if (gamepad1.rightStickButtonWasPressed()) writeLog(gamepadLog + "RIGHT_STICK_BUTTON1:" + gamepad1.right_stick_button);

        if (gamepad1.optionsWasPressed()) writeLog(gamepadLog + "OPTIONS1:" + gamepad1.options);

        writeLog("STICK:LEFT_STICK_X1:" + gamepad1.left_stick_x);
        writeLog("STICK:LEFT_STICK_Y1:" + gamepad1.left_stick_y);

        writeLog("STICK:RIGHT_STICK_X1:" + gamepad1.left_stick_x);
        writeLog("STICK:RIGHT_STICK_Y1:" + gamepad1.left_stick_y);

        writeLog("TRIGGER:LEFT_TRIGGER1:" + gamepad1.left_trigger);
        writeLog("TRIGGER:RIGHT_TRIGGER1:" + gamepad1.right_trigger);
    }

    public void checkForGamepadInputD2() {
        if (gamepad2.aWasPressed()) writeLog(gamepadLog + "A2:" + gamepad2.a);
        if (gamepad2.bWasPressed()) writeLog(gamepadLog + "B2:" + gamepad2.b);
        if (gamepad2.xWasPressed()) writeLog(gamepadLog + "X2:" + gamepad2.x);
        if (gamepad2.yWasPressed()) writeLog(gamepadLog + "Y2:" + gamepad2.y);

        if (gamepad2.dpadUpWasPressed()) writeLog(gamepadLog + "DPAD_UP2:" + gamepad2.dpad_up);
        if (gamepad2.dpadDownWasPressed()) writeLog(gamepadLog + "DPAD_DOWN2:" + gamepad2.dpad_down);
        if (gamepad2.dpadRightWasPressed()) writeLog(gamepadLog + "DPAD_RIGHT2:" + gamepad2.dpad_right);
        if (gamepad2.dpadLeftWasPressed()) writeLog(gamepadLog + "DPAD_LEFT2:" + gamepad2.dpad_left);

        if (gamepad2.startWasPressed()) writeLog(gamepadLog + "START2:" + gamepad2.start);
        if (gamepad2.backWasPressed()) writeLog(gamepadLog + "BACK2:" + gamepad2.back);

        if (gamepad2.leftBumperWasPressed()) writeLog(gamepadLog + "LEFT_BUMPER2:" + gamepad2.left_bumper);
        if (gamepad2.rightBumperWasPressed()) writeLog(gamepadLog + "RIGHT_BUMPER2:" + gamepad2.right_bumper);

        if (gamepad2.leftStickButtonWasPressed()) writeLog(gamepadLog + "LEFT_STICK_BUTTON2:" + gamepad2.left_stick_button);
        if (gamepad2.rightStickButtonWasPressed()) writeLog(gamepadLog + "RIGHT_STICK_BUTTON2:" + gamepad2.right_stick_button);

        if (gamepad2.optionsWasPressed()) writeLog(gamepadLog + "OPTIONS2:" + gamepad2.options);

        writeLog("STICK:LEFT_STICK_X2:" + gamepad2.left_stick_x);
        writeLog("STICK:LEFT_STICK_Y2:" + gamepad2.left_stick_y);

        writeLog("STICK:RIGHT_STICK_X2:" + gamepad2.right_stick_x);
        writeLog("STICK:RIGHT_STICK_Y2:" + gamepad2.right_stick_y);

        writeLog("TRIGGER:LEFT_TRIGGER2:" + gamepad2.left_trigger);
        writeLog("TRIGGER:RIGHT_TRIGGER2:" + gamepad2.right_trigger);
    }

    // MOTOR DATA FUNCTION
    public void checkForMotors() {
        for (MotorNames motorName : MotorNames.values()) {
            String name = String.valueOf(motorName);
            MotorComponent motor = robot.getMotorComponent(name);
            cycleBuffer.append("MOTOR:").append(motorName).append(":POWER").append(motor.getPower()).append('\n');
            cycleBuffer.append("MOTOR:").append(motorName).append(":VELOCITY").append(motor.getVelocity()).append('\n');
            cycleBuffer.append("MOTOR:").append(motorName).append(":CURRENT").append(motor.getCurrent()).append('\n');
            cycleBuffer.append("MOTOR:").append(motorName).append(":POSITION").append(motor.getPosition()).append('\n');
            cycleBuffer.append("MOTOR:").append(motorName).append(":ABS_POSITION").append(motor.getAbsolutePosition()).append('\n');
        }
    }

    public void checkForServos() {
        for (ServoNames servoName : ServoNames.values()) {
            ServoComponent servo = robot.getServoComponent(String.valueOf(servoName));
            cycleBuffer.append("SERVO:").append(servoName).append(":POSITION").append(servo.getPosition()).append('\n');
        }
    }

    public void checkForPosition() {
        cycleBuffer.append("ROBOT_POSITION:").append(robot.getCurrentPose()).append('\n');
    }

    public void checkForVoltage() {
        cycleBuffer.append("GENERAL_VOLTAGE:").append(controlHubVoltageSensor.getVoltage()).append('\n');
    }
}