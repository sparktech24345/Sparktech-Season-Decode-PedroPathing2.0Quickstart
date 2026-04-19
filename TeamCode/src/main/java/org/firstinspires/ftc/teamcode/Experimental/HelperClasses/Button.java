package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.ButtonPressedEvent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.ButtonReleasedEvent;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Button {
    private final BooleanSupplier condB;
    private final DoubleSupplier condD;
    private boolean WasPressed = false;
    private final ElapsedTime HoldTimer = new ElapsedTime();


    public boolean toggled = false;
    public boolean held = false;
    public double  HoldTimeMs = 0;
    public boolean pressed = false;
    public boolean released = false;

    public Button(BooleanSupplier execCond) {
        this.condB = execCond;
        this.condD = null;
    }

    public Button(DoubleSupplier execCond) {
        this.condB = null;
        this.condD = execCond;
    }

    public double raw() {
        try {
            return condD.getAsDouble();
        } catch (NullPointerException e1) {
            try {
                return eval(condB.getAsBoolean());
            } catch (NullPointerException e2) {
                return 0;
            }
        }
    }

    public void update() {
        boolean input = (condB == null ? evalForTrigger((condD == null ? 0 : condD.getAsDouble())) : condB.getAsBoolean());
        pressed = !WasPressed && input;
        released = WasPressed && !input;
        held = input;
        WasPressed = input;
        if (pressed) {
            HoldTimer.reset();
            toggled = !toggled;
            ComplexOpMode.publicEventBus.emit(new ButtonPressedEvent(this));
        }
        else if (released) {
            HoldTimeMs = HoldTimer.milliseconds();
            toggled = !toggled;
            ComplexOpMode.publicEventBus.emit(new ButtonReleasedEvent(this));
        }
    }
}
