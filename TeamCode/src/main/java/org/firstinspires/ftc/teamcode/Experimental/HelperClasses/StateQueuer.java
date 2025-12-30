package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.Vector;


public class StateQueuer {

    private final Queue<Action> actionQueue = new ArrayDeque();
    private final Vector<Action> instantActions = new Vector<>();
    private boolean prevDone;

    public int getLen() {
        return actionQueue.size();
    }

    public StateQueuer addAction(Action action) {
        actionQueue.add(action);
        return this;
    }

    public StateQueuer executeNow(Action action) {
        instantActions.add(action);
        return this;
    }

    public boolean isEmpty() { return actionQueue.isEmpty(); }

    public void update() {
        for (int i = 0; i < instantActions.size();) {
            Action action = instantActions.get(i);
            action.update();
            if (action.finished()) instantActions.remove(i);
            else ++i;
        }

        for (Action action : actionQueue) {
            action.update();
            prevDone = action.finished();
            if (!prevDone) break;
        }

        while (!actionQueue.isEmpty()) {
            Action lastAction = actionQueue.peek();
            if (lastAction != null && lastAction.finished()) actionQueue.poll();
            else break;
        }
    }
}





















//          if (actionQueue.peek().finished()) actionQueue.poll();
