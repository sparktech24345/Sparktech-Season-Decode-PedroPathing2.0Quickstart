package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.StateQueuer;

public class ActionSequence extends Action {

    protected StateQueuer queue = new StateQueuer();

    public ActionSequence(Action... actions) {
        super();
        this.DoneCondition = () -> queue.isEmpty();
        this.Execution = () -> queue.update();
        this.StartCondition = () -> true;
        for (Action action : actions) {
            queue.addAction(action);
        }
    }


}
