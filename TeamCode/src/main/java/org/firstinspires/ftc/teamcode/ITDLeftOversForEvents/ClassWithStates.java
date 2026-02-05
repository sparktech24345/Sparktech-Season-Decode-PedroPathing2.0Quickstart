package org.firstinspires.ftc.teamcode.ITDLeftOversForEvents;

import static org.firstinspires.ftc.teamcode.ITDLeftOversForEvents.OrganizedPositionStorage.*;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ClassWithStates {

    public static enum intakeStates{
        noStateSet,
        intakeExtended4out4,
        intakeExtended3out4,
        intakeExtended2out4,
        intakeExtended1out4,
        intakeRetracted,
    }
    public static intakeStates intakeState = intakeStates.noStateSet;

    public static enum intakeCabinStates{
        noStateSet,
        intakeCabinDownCollecting,
        intakeCabinDownOutputting,
        intakeCabinDownStandStill,
        intakeCabinTransferPosition,
        intakeCabinFullInBot,
        intakeCabinFullInBotOutputting, isInTransferToIntakeCabinDownCollecting,
    }
    public static intakeCabinStates intakeCabinState = intakeCabinStates.noStateSet;

    public static enum outtakeStates{
        noStateSet,
        outtakeSpecimenHang,
        outtakeBasket,
        outtakeLowerBasket,
        outtakeWallPickUpNormal,
        outtakeWallPickUpNew,
        outtakeTransfer,
        outtakeStandBy,
        outtakeStandByWithoutExtensions,
        autoPark
    }
    public static outtakeStates outtakeState = outtakeStates.noStateSet;

    public static enum colorSensorOutty{
        noSample,
        correctSample,
        wrongSample,
    }

    public static colorSensorOutty currentStateOfSampleInIntake = colorSensorOutty.noSample;
    public static enum colorList{
        teamNotSet,
        red,
        blue,
        yellow,
    }

    public static colorList currentTeam = colorList.teamNotSet;


    public static void intakeExtended4out4(){
        intakeState = intakeStates.intakeExtended4out4;
        intakeExtendMotorTargetPos = 585;
        intakeGravitySubtractor =8;
    }
    public static void intakeExtended3out4(){
        intakeState = intakeStates.intakeExtended3out4;
        intakeExtendMotorTargetPos = 377;
        intakeGravitySubtractor =6;
    }
    public static void intakeExtended2out4(){
        intakeState = intakeStates.intakeExtended2out4;
        intakeExtendMotorTargetPos = 245;
        intakeGravitySubtractor =4;
    }
    public static void intakeExtended1out4(){
        intakeState = intakeStates.intakeExtended1out4;
        intakeExtendMotorTargetPos = 112;
        intakeGravitySubtractor =2;
    }
    public static void intakeRetracted(){
        intakeState = intakeStates.intakeRetracted;
        intakeExtendMotorTargetPos = 0;
        intakeGravitySubtractor =0;
    }



    public static void intakeCabinDownCollecting(){
        intakeCabinState = intakeCabinStates.intakeCabinDownCollecting;
        intakePivotServoPos = intakePivotServoPickupPos;
        isAfterTakingTakeySpiny = true;
        intakeSpinMotorPow = -1;
    }
    public static void intakeCabinDownOutputting(){
        intakeCabinState = intakeCabinStates.intakeCabinDownOutputting;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = 0.6;
    }
    public static void intakeCabinALittleBitUpStandStill(){
        intakeCabinState = intakeCabinStates.intakeCabinDownStandStill;
        intakePivotServoPos = intakePivotServoPickupPos-5;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinALittleBitUpOutputting(){
        intakeCabinState = intakeCabinStates.intakeCabinDownStandStill;
        intakePivotServoPos = intakePivotServoPickupPos-5;
        intakeSpinMotorPow = 0.9;
    }
    public static void intakeCabinTransferPosition(){
        intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
        intakePivotServoPos = intakePivotServoTransferPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinTransferPositionWithPower(){
        intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
        intakePivotServoPos = intakePivotServoTransferPos;
        shouldStopIntakeCabinSpinningAfterTakig = true;
        shouldStopIntakeCabinSpinningAfterTakigTimer = System.currentTimeMillis();
        intakeSpinMotorPow = -0.5;
    }
    public static void intakeCabinFullInBot(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBot;
        intakePivotServoPos = intakePivotServoOutputTruBotPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinFullInBotOutputting(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBotOutputting;
        intakePivotServoPos = intakePivotServoOutputTruBotPos;
        intakeSpinMotorPow = 0.9;
    }


    public static void outtakePark(){
        outtakeState = outtakeStates.autoPark;
        outtakePivotServoPos = outtakePivotServoParkedPos;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakeExtendMotorTargetPos = outtakeMotorParkedPos;
    }

    public static void outtakeSpecimenHang(){
        outtakeState = outtakeStates.outtakeSpecimenHang;
        outtakePivotServoPos = outtakePivotServoHighRungHangPos;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakeExtendMotorTargetPos = outtakeSliderSpecimenHangPos;
    }
    public static void outtakeBasket(){
        outtakeState = outtakeStates.outtakeBasket;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakePivotServoPos = outtakePivotServoBasketPos;
        if(!isInLowerBasketState) outtakeExtendMotorTargetPos = outtakeMotorMaxPos;
        else outtakeExtendMotorTargetPos = outtakeMotorMaxPosLowerBasket;
    }

    public static void outtakeLowerBasket(){
        outtakeState = outtakeStates.outtakeLowerBasket;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakePivotServoPos = outtakePivotServoBasketPos;
        outtakeExtendMotorTargetPos = outtakeMotorMaxPosLowerBasket;
    }

    public static void outtakeWallPickUpNew(){
        outtakeState = outtakeStates.outtakeWallPickUpNew;

        isInNeedToGoToSpecimenTransferPos = true;
        needsToExtraExtend = true;
        outtakeClawServoPos = outtakeClawServoExtendedPos;

        outtakeIsInNeedToExtraExtendClawTimer = System.currentTimeMillis();
        outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
    }
    public static void autoOuttakeWallPickUpNew(){
        outtakeState = outtakeStates.outtakeWallPickUpNew;
        outtakePivotServoPos = outtakePivotServoWallPickupPos;
        isInNeedToGoToSpecimenTransferPos = true;
        needsToExtraExtend = true;
        outtakeClawServoPos = outtakeClawServoExtendedPos;

        outtakeIsInNeedToExtraExtendClawTimer = System.currentTimeMillis();
        outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
    }

    public static void outtakeTransfer(){
        outtakeState = outtakeStates.outtakeTransfer;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakePivotServoPos = outtakePivotServoTransferPos;

        isOuttakeInPositionToGoDown = true;
        beforeOuttakeGoDownTimer = System.currentTimeMillis();
    }
    public static void autoOuttakeTransfer(){
        outtakeState = outtakeStates.outtakeTransfer;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakePivotServoPos = outtakePivotServoTransferPos;
        outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
        isOuttakeInPositionToGoDown = true;
        beforeOuttakeGoDownTimer = System.currentTimeMillis();
    }


    public static void outtakeStandByWithoutExtensions(){
        outtakeState = outtakeStates.outtakeStandByWithoutExtensions;
        outtakePivotServoPos = outtakePivotServoStandByPos;
        outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
    }

    public static colorSensorOutty ColorCompare(NormalizedRGBA colors, colorList currentTeam,boolean isYellowSampleNotGood){

        if(!(colors.red > 0.006 || colors.blue > 0.004)) return colorSensorOutty.noSample;
        colorList color=colorList.teamNotSet;

        if (colors.red > colors.blue && colors.red > colors.green)
            color = colorList.red;
        if (colors.blue > colors.red && colors.blue > colors.green)
            color = colorList.blue;
        if (colors.green > colors.blue && colors.green > colors.red)
            color = colorList.yellow;

        colorList wrongSampleType = colorList.teamNotSet;
        if(currentTeam == colorList.blue) wrongSampleType = colorList.red;
        if(currentTeam == colorList.red) wrongSampleType = colorList.blue;

        if(color == wrongSampleType) return colorSensorOutty.wrongSample;
        if(isYellowSampleNotGood && color == colorList.yellow) return colorSensorOutty.wrongSample;
        else return colorSensorOutty.correctSample;
    }

    public static void initStates() {
        outtakeStandByWithoutExtensions();
        intakeCabinFullInBot();
        intakeRetracted();
    }

}