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
            return false;
        }

        for (int i = 0; i < MAX_SIZE; i++) {
            if (queue[i] == BallColorSet_Decode.NoBall) {
                queue[i] = color;
                return true;
            }
        }
        return false;
    }


    public BallColorSet_Decode pull() {
        BallColorSet_Decode first = queue[0];

        // Shift everything forward
        for (int i = 0; i < MAX_SIZE - 1; i++) {
            queue[i] = queue[i + 1];
        }

        queue[MAX_SIZE - 1] = BallColorSet_Decode.NoBall;
        return first;
    }


    public void clearQueue() {
        for (int i = 0; i < MAX_SIZE; i++) {
            queue[i] = BallColorSet_Decode.NoBall;
        }
    }

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

    public boolean push(BallColorSet_Decode color) {
        if (color == BallColorSet_Decode.NoBall) {
            return false;
        }

        for (int i = MAX_SIZE - 1; i > 0; i--) {
            queue[i] = queue[i - 1];
        }

        // Insert at the front
        queue[0] = color;

        return true;
    }

}
