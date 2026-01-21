package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.MainConfig;

@TeleOp(name = "Main TeleOp Red", group = "AAA")
public class MainTeleOpRed extends MainTeleOpBlue {
    public void makeConfig(){
        cfg = new MainConfig(MainConfig.Configs.Red);
    }
}
