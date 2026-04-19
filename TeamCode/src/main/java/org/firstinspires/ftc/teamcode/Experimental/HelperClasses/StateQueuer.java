package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;


import static org.firstinspires.ftc.teamcode.Experimental.HelperClasses.ComplexOpMode.publicTelemetry;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.Action;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Vector;
import java.util.stream.Stream;


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

    public StateQueuer addToQueue(Action ...actions) {
        Stream<Action> actionStream = Arrays.stream(actions);
        actionStream.forEach(this::addAction);
        return this;
    }

    public StateQueuer executeNow(Action action) {
        instantActions.add(action);
        return this;
    }

    public StateQueuer executeNow(Action ...actions) {
        Collections.addAll(instantActions, actions);
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

        while (!actionQueue.isEmpty()) {
            Action action = actionQueue.get(0);
            action.update();
            isDone = action.finished();
            if (isDone) actionQueue.remove(0);
            else break;
        }
    }

    public void telemetry() {
        publicTelemetry.addData("queue len", actionQueue.size());
        publicTelemetry.addData("instants len", instantActions.size());
        publicTelemetry.addData("isEmpty", isEmpty());
        if (actionQueue.isEmpty()) return;
        actionQueue.get(0).telemetry();
    }

    public void clearQueue() {
        actionQueue.clear();
    }

    public void stopAll() {
        actionQueue.clear();
        instantActions.clear();
    }
}
