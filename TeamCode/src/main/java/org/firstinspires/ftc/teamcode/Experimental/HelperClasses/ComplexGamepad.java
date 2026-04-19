package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public enum ComplexGamepad {
    A1,
    B1,
    X1,
    Y1,

    DPAD_UP1,
    DPAD_DOWN1,
    DPAD_RIGHT1,
    DPAD_LEFT1,

    START1,
    BACK1,

    LEFT_STICK_X1,
    LEFT_STICK_Y1,

    RIGHT_STICK_X1,
    RIGHT_STICK_Y1,

    LEFT_TRIGGER1,
    RIGHT_TRIGGER1,

    LEFT_BUMPER1,
    RIGHT_BUMPER1,


    A2,
    B2,
    X2,
    Y2,

    DPAD_UP2,
    DPAD_DOWN2,
    DPAD_RIGHT2,
    DPAD_LEFT2,

    START2,
    BACK2,

    LEFT_STICK_X2,
    LEFT_STICK_Y2,

    RIGHT_STICK_X2,
    RIGHT_STICK_Y2,

    LEFT_TRIGGER2,
    RIGHT_TRIGGER2,

    LEFT_BUMPER2,
    RIGHT_BUMPER2;

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    private Button button;
    private static boolean init_ = false;

    public void populate(Button button) {
        this.button = button;
    }

    public static boolean isInit() {
        return init_;
    }

    public static void init(Gamepad gpad1, Gamepad gpad2) {
        init_ = true;
        SetGamepads(gpad1, gpad2);
    }

    public static void SetGamepads(Gamepad gpad1, Gamepad gpad2) {
        ComplexGamepad.gamepad1 = gpad1;
        ComplexGamepad.gamepad2 = gpad2;

        ComplexGamepad.A1.populate(new Button(() -> gpad1.a));
        ComplexGamepad.B1.populate(new Button(() -> gpad1.b));
        ComplexGamepad.X1.populate(new Button(() -> gpad1.x));
        ComplexGamepad.Y1.populate(new Button(() -> gpad1.y));

        ComplexGamepad.DPAD_UP1.populate(new Button(() -> gpad1.dpad_up));
        ComplexGamepad.DPAD_DOWN1.populate(new Button(() -> gpad1.dpad_down));
        ComplexGamepad.DPAD_RIGHT1.populate(new Button(() -> gpad1.dpad_right));
        ComplexGamepad.DPAD_LEFT1.populate(new Button(() -> gpad1.dpad_left));

        ComplexGamepad.START1.populate(new Button(() -> gpad1.start));
        ComplexGamepad.BACK1.populate(new Button(() -> gpad1.back));

        ComplexGamepad.LEFT_STICK_X1.populate(new Button(() -> gpad1.left_stick_x));
        ComplexGamepad.LEFT_STICK_Y1.populate(new Button(() -> gpad1.left_stick_y));

        ComplexGamepad.RIGHT_STICK_X1.populate(new Button(() -> gpad1.right_stick_x));
        ComplexGamepad.RIGHT_STICK_Y1.populate(new Button(() -> gpad1.right_stick_y));

        ComplexGamepad.LEFT_TRIGGER1.populate(new Button(() -> gpad1.left_trigger));
        ComplexGamepad.RIGHT_TRIGGER1.populate(new Button(() -> gpad1.right_trigger));

        ComplexGamepad.LEFT_BUMPER1.populate(new Button(() -> gpad1.left_bumper));
        ComplexGamepad.RIGHT_BUMPER1.populate(new Button(() -> gpad1.right_bumper));


        ComplexGamepad.A2.populate(new Button(() -> gpad2.a));
        ComplexGamepad.B2.populate(new Button(() -> gpad2.b));
        ComplexGamepad.X2.populate(new Button(() -> gpad2.x));
        ComplexGamepad.Y2.populate(new Button(() -> gpad2.y));

        ComplexGamepad.DPAD_UP2.populate(new Button(() -> gpad2.dpad_up));
        ComplexGamepad.DPAD_DOWN2.populate(new Button(() -> gpad2.dpad_down));
        ComplexGamepad.DPAD_RIGHT2.populate(new Button(() -> gpad2.dpad_right));
        ComplexGamepad.DPAD_LEFT2.populate(new Button(() -> gpad2.dpad_left));

        ComplexGamepad.START2.populate(new Button(() -> gpad2.start));
        ComplexGamepad.BACK2.populate(new Button(() -> gpad2.back));

        ComplexGamepad.LEFT_STICK_X2.populate(new Button(() -> gpad2.left_stick_x));
        ComplexGamepad.LEFT_STICK_Y2.populate(new Button(() -> gpad2.left_stick_y));

        ComplexGamepad.RIGHT_STICK_X2.populate(new Button(() -> gpad2.right_stick_x));
        ComplexGamepad.RIGHT_STICK_Y2.populate(new Button(() -> gpad2.right_stick_y));

        ComplexGamepad.LEFT_TRIGGER2.populate(new Button(() -> gpad2.left_trigger));
        ComplexGamepad.RIGHT_TRIGGER2.populate(new Button(() -> gpad2.right_trigger));

        ComplexGamepad.LEFT_BUMPER2.populate(new Button(() -> gpad2.left_bumper));
        ComplexGamepad.RIGHT_BUMPER2.populate(new Button(() -> gpad2.right_bumper));
    }

    public Button get() {
        return button;
    }

    // MUST BE INCLUDED IN MAIN LOOP FOR THE BOOLEANS TO WORK
    public static void update() {
        for (ComplexGamepad b : ComplexGamepad.values()) {
            if (b == null) continue;
            b.button.update();
        }
    }
}
