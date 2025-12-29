package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.StateQueuer;

import java.util.ArrayDeque;
import java.util.Queue;

public class ActionSequence extends Action {

    protected StateQueuer queue = new StateQueuer();

    public ActionSequence(Action... actions) {
        for (Action action : actions) {
            queue.addAction(action);
        }
        this.DoneCondition = () -> queue.isEmpty();
        this.Execution = () -> queue.update();
        this.ExecutionCondition = () -> true;
    }


}
