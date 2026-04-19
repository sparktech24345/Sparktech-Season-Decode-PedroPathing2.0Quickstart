package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Button;

public class ButtonToggledEvent extends Event {
    private Button button;
    public ButtonToggledEvent(Button button) {
        this.button = button;
    }

    public Button getButton() {
        return button;
    }
}
