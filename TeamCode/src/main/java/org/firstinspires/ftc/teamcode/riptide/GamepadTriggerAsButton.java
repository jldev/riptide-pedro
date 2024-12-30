package org.firstinspires.ftc.teamcode.riptide;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class GamepadTriggerAsButton extends Button {

    private GamepadTrigger mTrigger;
    private double mTriggerThreshold;

    public GamepadTriggerAsButton(GamepadEx gamepad, GamepadKeys.Trigger trigger, double threshold){
        mTrigger = new GamepadTrigger(gamepad, trigger);
        mTriggerThreshold = threshold;
    }

    /**
     * Gets the value of the trigger an compares it to the threshold returns as a boolean.
     *
     * @return boolean True if trigger value is greater than threshold
     */
    @Override
    public boolean get() {
        return mTrigger.get() > mTriggerThreshold;
    }

    public double getTriggerValue() {
        return mTrigger.get();
    }
}
