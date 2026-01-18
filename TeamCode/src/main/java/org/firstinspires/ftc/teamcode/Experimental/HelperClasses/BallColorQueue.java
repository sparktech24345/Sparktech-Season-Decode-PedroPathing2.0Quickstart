package org.firstinspires.ftc.teamcode.Experimental.HelperClasses;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums.BallColorSet_Decode;

public class BallColorQueue {

    private static final int MAX_SIZE = 15;
    private final BallColorSet_Decode[] queue = new BallColorSet_Decode[MAX_SIZE];

    public BallColorQueue() {
        clearQueue();
    }

    /**
     * Adds a color to the first empty position (NoBall),
     * NOT pushing existing elements forward.
     */
    public boolean add(BallColorSet_Decode color) {
        if (color == BallColorSet_Decode.NoBall) {
            return false; // do not add NoBall
        }

        for (int i = 0; i < MAX_SIZE; i++) {
            if (queue[i] == BallColorSet_Decode.NoBall) {
                queue[i] = color;
                return true;
            }
        }
        return false; // queue is full
    }

    /**
     * Pulls the first element, removes it,
     * and shifts remaining elements forward.
     */
    public BallColorSet_Decode pull() {
        BallColorSet_Decode first = queue[0];

        // Shift everything forward
        for (int i = 0; i < MAX_SIZE - 1; i++) {
            queue[i] = queue[i + 1];
        }

        queue[MAX_SIZE - 1] = BallColorSet_Decode.NoBall;
        return first;
    }

    /**
     * Clears the queue to all NoBall
     */
    public void clearQueue() {
        for (int i = 0; i < MAX_SIZE; i++) {
            queue[i] = BallColorSet_Decode.NoBall;
        }
    }

    // Optional helper for debugging
    public BallColorSet_Decode[] getSnapshot() {
        return queue.clone();
    }
    public void spitOutQueueInTelemetry() {
        for (int i = 0; i < MAX_SIZE; i++) {
            RobotController.telemetry.addData(
                    "Ball number in queue " + i,
                    queue[i]
            );
        }
    }

    /**
     * Pushes a color to the front of the queue,
     * shifting all existing elements one position back.
     * The last element is discarded if the queue is full.
     */
    public boolean push(BallColorSet_Decode color) {
        if (color == BallColorSet_Decode.NoBall) {
            return false; // do not push NoBall
        }

        // Shift everything back (from end to start)
        for (int i = MAX_SIZE - 1; i > 0; i--) {
            queue[i] = queue[i - 1];
        }

        // Insert at the front
        queue[0] = color;

        return true;
    }

}
