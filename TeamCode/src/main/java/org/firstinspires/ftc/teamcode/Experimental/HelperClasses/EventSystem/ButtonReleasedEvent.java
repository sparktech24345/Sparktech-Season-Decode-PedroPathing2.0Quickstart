package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Button;

public class ButtonReleasedEvent extends Event {
    private Button button;
    public ButtonReleasedEvent(Button button) {
        this.button = button;
    }

    public Button getButton() {
        return button;
    }
}
