package org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experimental.MainOpModes.Configs.Config;

@TeleOp(name = "Main TeleOp Red", group = "AAA")
public class MainTeleOpRed extends MainTeleOpBlue {
    public void makeConfig() {
        cfg = Config.getConfig("red");
    }
}
