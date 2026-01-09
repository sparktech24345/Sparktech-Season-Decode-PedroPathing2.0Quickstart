package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions;

import java.util.function.BooleanSupplier;

public class GeneralAction extends Action {

    public GeneralAction(Runnable exec) {
        super();
        this.Execution = exec;
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

