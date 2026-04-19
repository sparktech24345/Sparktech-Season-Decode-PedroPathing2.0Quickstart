package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem;

import java.util.EventObject;

@FunctionalInterface
public interface EventListener <T extends Event> {
    void onEvent(T event);
}
