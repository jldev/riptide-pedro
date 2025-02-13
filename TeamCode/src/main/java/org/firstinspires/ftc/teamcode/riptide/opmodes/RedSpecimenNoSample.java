package org.firstinspires.ftc.teamcode.riptide.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.riptide.Riptide;

@Config
@Autonomous(group = "drive", name = "Specimen No Sample")
public class RedSpecimenNoSample extends CommandOpMode {
    RiptideAuto riptideAuto;
    @Override
    public void initialize() {
        riptideAuto = new RiptideAuto(this, Riptide.FieldPos.AU, Riptide.AllianceColor.RED, Riptide.Target.SPECIMENS);
        riptideAuto.riptide.pushSamples = false;
    }

    @Override
    public void run(){
        riptideAuto.run();
        super.run();
    }
}
