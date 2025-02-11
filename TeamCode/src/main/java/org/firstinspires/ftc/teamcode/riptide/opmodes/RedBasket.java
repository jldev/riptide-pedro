package org.firstinspires.ftc.teamcode.riptide.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.riptide.Riptide;

@Config
@Autonomous(group = "drive", name = "Red Basket")
public class RedBasket extends CommandOpMode {
    RiptideAuto riptideAuto;
    @Override
    public void initialize() {
        riptideAuto = new RiptideAuto(this, Riptide.FieldPos.AU, Riptide.AllianceColor.RED, Riptide.Target.SAMPLES);
        riptideAuto.riptide.pushSamples = true;
    }

    @Override
    public void run(){
        riptideAuto.run();
        super.run();
    }
}
