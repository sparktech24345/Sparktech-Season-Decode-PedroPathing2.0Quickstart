package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.GlobalStorage.*;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.States.State;

public class StateAction extends Action {

    public StateAction(State state) {
        super();
        this.Execution = () -> state.owner().setState(state);
    }
}