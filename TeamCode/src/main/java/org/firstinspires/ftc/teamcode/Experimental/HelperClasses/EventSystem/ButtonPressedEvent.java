package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Button;

public class ButtonPressedEvent extends Event {
    private Button button;
    public ButtonPressedEvent(Button button) {
        this.button = button;
    }

    public Button getButton() {
        return button;
    }
}
