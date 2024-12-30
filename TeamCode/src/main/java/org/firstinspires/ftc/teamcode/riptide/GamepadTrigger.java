package org.firstinspires.ftc.teamcode.riptide;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class GamepadTrigger {

    private GamepadKeys.Trigger mTrigger;
    private GamepadEx mGamepad;
    private double mTriggerThreshold;

    public GamepadTrigger(GamepadEx gamepad, GamepadKeys.Trigger trigger){
        mTrigger = trigger;
        mGamepad = gamepad;
    }

    /**
     * Gets the value of the trigger
     *
     * @return double value of the trigger
     */
    public double get() {
        return mGamepad.getTrigger(mTrigger);
    }
}
