package riptide.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import riptide.Riptide;

@Config
@Autonomous(group = "drive", name = "Basket")
public class Basket extends CommandOpMode {
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
