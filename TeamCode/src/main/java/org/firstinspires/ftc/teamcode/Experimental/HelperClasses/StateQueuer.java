package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;

import java.util.ArrayDeque;
import java.util.Queue;


public class StateQueuer {

    private final Queue<Action> actionQueue = new ArrayDeque();
    private boolean prevDone;

    public int getLen() {
        return actionQueue.size();
    }

    public StateQueuer addAction(Action action) {
        actionQueue.add(action);
        return this;
    }

    public boolean isEmpty() { return actionQueue.isEmpty(); }

    public void update() {
        prevDone = true;
        for (Action action : actionQueue) {
            action.update(prevDone);
            if (action.waits() && !prevDone) break;
            prevDone = action.finished();
        }
        while (!actionQueue.isEmpty()) {
            if (actionQueue.peek().finished()) actionQueue.poll();
            else break;
        }
    }
}





















//          if (actionQueue.peek().finished()) actionQueue.poll();
