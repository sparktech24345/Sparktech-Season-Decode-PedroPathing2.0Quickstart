package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.DecodeEnums;

public enum MotifSequence {
    Undefined,
    Green_Purple_Purple,
    Purple_Purple_Green,
    Purple_Green_Purple;

    public static MotifSequence getNextSequence(MotifSequence inputSec){
        if(inputSec == Green_Purple_Purple) return Purple_Purple_Green;
        else if(inputSec == Purple_Purple_Green) return Purple_Green_Purple;
        else if(inputSec == Purple_Green_Purple) return Green_Purple_Purple;

        //we should not get here
        return Undefined;
    }
}
