package org.firstinspires.ftc.teamcode.utils.AutoPIDTuner.TypesTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.AutoPIDTuner.TrainAutoPIDBaseTeleopHandler;
import org.firstinspires.ftc.teamcode.utils.AutoPIDTuner.TunningTypes;

@TeleOp(name = "AutoPIDHeavySlides", group = "AutoPIDS")
@Disabled
public class AutoPIDHeavySlides extends TrainAutoPIDBaseTeleopHandler {
    public AutoPIDHeavySlides(){
        super(TunningTypes.heavySlidesTuning);
    }
}
