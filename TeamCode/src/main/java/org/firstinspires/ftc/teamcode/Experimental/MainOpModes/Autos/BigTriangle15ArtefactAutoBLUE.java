package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Autos;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.DelayAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.GeneralAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.MoveAction;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Actions.StateAction;
import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@Autonomous(name="Big Triangle 15 Artefact Auto BLUE", group = "BBB")
public class BigTriangle15ArtefactAutoBLUE extends BigTriangle12ArtefactAuto {
    @Override
    public void makeSortedAuto(){
        robot.addToQueue(
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(bezierCurveShooterPoseTipOfTriangle),//false,true,false),
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1200),
//                new GeneralAction(prepToFireSortedBall),
//                new DelayAction(400),
//                new GeneralAction(fireSortedBall),
//                new DelayAction(500),
//                new GeneralAction(fireSortedBall),
//                new DelayAction(500),
//                new GeneralAction(fireSortedBall),
//                new DelayAction(500),

                ///second now

                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                new MoveAction(second_row_ready),
                new MoveAction(second_row_done),

                new GeneralAction(new Runnable() {
                    @Override
                    public void run() {
                        shouldFire = true;
                        shouldMoveIntakeServo = false;
                    }
                }),
                new MoveAction(bezierCurveShooterPoseTipOfTriangle),
                new GeneralAction(prepToFireSortedBall),
                new StateAction("IntakeMotor","FULL"),
                new DelayAction(200),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1400),
                // end of first row firing



                /// second row + lever on repeat
                new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                //new MoveAction(gateCollectPose,gateCollectHelperPoint,false,true),//gateCollectHelperPoint,false),
                new MoveAction(leverCollectPose),
                new DelayAction(1200),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(bezierCurveShooterPoseTipOfTriangle,gateCollectHelperPointForReverse,true,true),
                new DelayAction(350),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1800),
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),

                /// second row + lever on repeat
        new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                //new MoveAction(gateCollectPose,gateCollectHelperPoint,false,true),//gateCollectHelperPoint,false),
                new MoveAction(leverCollectPose),
                new DelayAction(1200),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(bezierCurveShooterPoseTipOfTriangle,gateCollectHelperPointForReverse,true,true),
                new DelayAction(350),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1800),
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff),


        /// second row + lever on repeat
        new StateAction("IntakeMotor","FULL"),
                new GeneralAction(increaseCollectNumber),
                new GeneralAction(turnOnIntakeServo),
                //new MoveAction(gateCollectPose,gateCollectHelperPoint,false,true),//gateCollectHelperPoint,false),
                new MoveAction(leverCollectPose),
                new DelayAction(1200),
                new GeneralAction(() -> {
                    shouldFire = true;
                    shouldMoveIntakeServo = false;
                }),
                new MoveAction(bezierCurveShooterPoseTipOfTriangle,gateCollectHelperPointForReverse,true,true),
                new DelayAction(350),
                new GeneralAction(fireUnsortedBalls),
                new DelayAction(1800),
                new StateAction("IntakeMotor","OFF"),
                new GeneralAction(turnStuffOff)
        );
    }
    @Override
    public void makeLeverAuto(){
        makeSortedAuto();
    }
}