package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import java.util.function.BooleanSupplier;

public class GeneralAction extends Action {

    public GeneralAction(boolean waitForPrevious, Runnable exec) {
        super(waitForPrevious);
        this.Execution = exec;
    }

    public GeneralAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }

    public GeneralAction setDoneCondition(BooleanSupplier done) {
        this.DoneCondition = done;
        return this;
    }
}
// example
//        robot.addToQueue(new GeneralAction(true, () -> { execution code })
//              .setExecutionCondition(() -> {
//                  return (condition when to start);
//              })
//              .setDoneCondition(() -> {
//                  return (condition when its done);
//              }));

