package org.firstinspires.ftc.teamcode.riptide;

import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwitchReader extends Button {

    public DigitalChannel magswitch = null;
    private boolean mHighTrue = false;
    public SwitchReader(HardwareMap hwMap, boolean highTrue, String switchName){
        magswitch = hwMap.digitalChannel.get(switchName);
        magswitch.setMode(DigitalChannel.Mode.INPUT);
        mHighTrue = highTrue;
    }
    @Override
    public boolean get() {
        if (mHighTrue) {
            return magswitch.getState();
        } else {
            return !magswitch.getState();
        }
    }


}
