package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Autonomous(name="Big Triangle Auto RED", group = "AAA")
public class BigTriangle15ArtefactAutoRED extends BigTriangle12ArtefactAuto {
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
    @Override
    public Pose convertPose(Pose pose){
        return new Pose(pose.getX(),-pose.getY(),- pose.getHeading());
    }
    private PathConstraints brutalConstraints = new PathConstraints( // copiate direct din exemplul Pedro, de verificat / corectat
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            2,
            10,
            0.6
    );
    @Override
    public void makeSortedAuto(){
        robot.addToQueue(
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect),
                new GeneralAction(prepToFireSortedBall),
                new DelayAction(400),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                new GeneralAction(fireSortedBall),
                new DelayAction(500),
                //new StateAction("IntakeMotor","OFF"),
                //new GeneralAction(turnStuffOff),


                /// collecting the closest row second now

                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(third_row_ready),
                //new MoveAction(third_row_intermediate),
                //new DelayAction(100),
                //new MoveAction(third_row_VERYintermediate),
                //new DelayAction(250),
                new MoveAction(third_row_done),
                new DelayAction(200),
                new MoveAction(leverPoseThirdRow),
                //new DelayAction(300),
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
                new MoveAction(big_triangle_shoot_third_collect),
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
                //new MoveAction(first_row_intermediate),
                new MoveAction(first_row_done,brutalConstraints),
                //new DelayAction(300),
                ///firing the 12th ball firing
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect), ///TODO might change to park
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
                new MoveAction(hp_ready,brutalConstraints),
                new MoveAction(hp_collect_pose,brutalConstraints),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(big_triangle_shoot_third_collect_with_park),
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