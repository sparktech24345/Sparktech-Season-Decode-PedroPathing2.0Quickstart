package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.RobotController.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.Vector;


public class StateQueuer {

    private final Vector<Action> actionQueue = new Vector<>();
    private final Vector<Action> instantActions = new Vector<>();
    private boolean isDone;

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

    public boolean isEmpty() { return actionQueue.isEmpty() && instantActions.isEmpty(); }

    public void update() {
        for (int i = 0; i < instantActions.size();) {
            Action action = instantActions.get(i);
            action.update();
            if (action.finished()) instantActions.remove(i);
            else ++i;
        }

        for (int i = 0; i < 1000 && !actionQueue.isEmpty(); ++i) {
            Action action = actionQueue.get(0);
            action.update();
            isDone = action.finished();
            if (isDone) actionQueue.remove(0);
            else break;
        }
    }

    public void telemetry() {
        telemetry.addData("queue len", actionQueue.size());
        telemetry.addData("instants len", instantActions.size());
        telemetry.addData("isEmpty", isEmpty());
        if (actionQueue.isEmpty()) return;
        actionQueue.get(0).telemetry();
    }
}
