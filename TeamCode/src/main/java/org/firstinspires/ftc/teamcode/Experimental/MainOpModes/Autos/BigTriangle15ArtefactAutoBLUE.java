package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;
@Disabled
@Autonomous(name="Big Triangle 15 Artefact Auto BLUE", group = "BBB")
public class BigTriangle15ArtefactAutoBLUE extends BigTriangle12ArtefactAuto {
    @Override
    public void makeSortedAuto(){
        robot.executeNow(new MoveAction(big_triangle_shoot_third_collect));
        robot.addToQueue(
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),

                /// collecting the closest row second now

                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(third_row_done),

                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(400),
                // end of first row firing



                /// second row + lever
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(second_row_ready),
                new MoveAction(second_row_done),
                new DelayAction(300),
                new MoveAction(leverPoseSecondRow),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_second_collect),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(400),
                /// end of second row firing




                /// beginning of third row collect
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(first_row_ready),
                new MoveAction(first_row_done),
                ///firing the 12th ball firing
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_second_collect), ///TODO might change to park
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(400),
                /// end of 12 ball  row firing


                /// 15th ball row
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),

                new MoveAction(hp_collect_pose),
                new DelayAction(150),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect_with_park_180),
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(300),
                /// end of 15 ball  row firing




                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff)
        );
    }
    @Override
    public void makeLeverAuto(){
        makeSortedAuto();
    }
}