package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.Event;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem.EventListener;

import java.util.function.BooleanSupplier;

public class Toggleable<T> {
    private final T first;
    private final T second;
    private boolean state; // false = first, true = second

    public Toggleable(T first, T second) {
        this.first = first;
        this.second = second;
        this.state = false;
    }
    public void toggle() {
        state = !state;
    }
    public <Te extends Event> void toggleOnEvent(Class<Te> type) {
        ComplexOpMode.publicEventBus.subscribe(type, (Te event) -> toggle());
    }
    public T get() {
        return state ? second : first;
    }
    public boolean isFirst() {
        return !state;
    }

    public boolean isSecond() {
        return state;
    }
    public void setFirst() {
        state = false;
    }

    public void setSecond() {
        state = true;
    }
}