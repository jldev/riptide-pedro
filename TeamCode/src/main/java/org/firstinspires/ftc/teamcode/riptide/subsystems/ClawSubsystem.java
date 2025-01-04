package org.firstinspires.ftc.teamcode.riptide.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.RiptideConstants;

public class ClawSubsystem extends SubsystemBase {

    private final Riptide mRiptide;

    private final CommandOpMode mOpMode;

    private final Servo yaw;
    private final Servo pitch;
    private final Servo grip;

    public enum ClawState {
        HOME,
        HANG,
        BASKET,
        SUB
    }

    public enum GripState {
        OPEN,
        CLOSED
    }
    public GripState mGripState;

    private double desiredYaw;
    private double desiredPitch;

    public ClawSubsystem(Riptide riptide, CommandOpMode commandOpMode, Servo yaw_1, Servo pitch_2, Servo grip_3) {
        mRiptide = riptide;
        mOpMode = commandOpMode;

        yaw = yaw_1;
        pitch = pitch_2;
        grip = grip_3;

        if(riptide.mOpModeType == Riptide.OpModeType.AUTO)
        {
            yaw.setPosition(RiptideConstants.CLAW_YAW_INIT);
            pitch.setPosition(RiptideConstants.CLAW_PITCH_INIT);
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE);
        }
        yaw.setPosition(yaw.getPosition());
        pitch.setPosition(pitch.getPosition());
        grip.setPosition(grip.getPosition());
    }

    @Override
    public void periodic() {

//        desiredYaw = (mHelix.gunnerOp.getLeftX() / 2) + .5;
//        desiredPitch = (mHelix.gunnerOp.getLeftY() / 2) + .5;





        //   GUNNER CONTROL


        desiredYaw = yaw.getPosition();
        desiredPitch = pitch.getPosition();

        if (Math.abs(mRiptide.gunnerOp.getLeftX()) > 0.1 || Math.abs(mRiptide.gunnerOp.getLeftY()) > 0.1) {
            desiredYaw = yaw.getPosition() + mRiptide.gunnerOp.getLeftX() * .045;
            desiredPitch = pitch.getPosition() + mRiptide.gunnerOp.getLeftY() * .045;
        }


        if (desiredYaw > 1.00)
            desiredYaw = 1.00;
        if (desiredYaw < 0.00)
            desiredYaw = 0.00;

        if (desiredPitch > .58)
            desiredPitch = .58;
        if (desiredPitch < 0.00)
            desiredPitch = 0.00;


//        if(mHelix.gunnerOp.getButton(GamepadKeys.Button.LEFT_BUMPER))
//        {
//            switch (desiredColor) {
//                case YELLOW: desiredColor = SampleColor.RED; break;
//                case RED: desiredColor = SampleColor.BLUE; break;
//                case BLUE: desiredColor = SampleColor.YELLOW; break;
//            }
//        }


        // After we get our positions from manual or auto - we set them here


//        mOpMode.telemetry.addData("Yaw:", desiredYaw);
//        mOpMode.telemetry.addData("Pitch:", desiredPitch);


        yaw.setPosition(desiredYaw);
        pitch.setPosition(desiredPitch);


//        if (mRiptide.gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .3)
//        {
//            mGripState = GripState.OPEN;
//            mRiptide.krakenEye.deployed = false;
//            mRiptide.krakenEye.hasSample = false;
//        } else if(!mRiptide.krakenEye.deployed)
//        {
//            mGripState = GripState.CLOSED;
//        }
//
//
//        if (mGripState == GripState.OPEN) {
//            grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);
//            mRiptide.krakenEye.hasSample = false;
//        } else {
//            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE);
//        }
    }

    //set mGripState and set servo accordingly, if its open kraken no haves sample
//    public void SetClawGripState(GripState state){
//        mGripState = state;
//        if (mGripState == GripState.OPEN) {
//            grip.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
//            mHelix.krakenEye.hasSample = false;
//        } else {
//            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
//        }
//    }


    //presets
    public void ChangeClawPositionTo(ClawState newClawState) {
        switch (newClawState) {
            case HOME:
                yaw.setPosition(RiptideConstants.YAW_HOME);
                pitch.setPosition(RiptideConstants.PITCH_HOME);
                break;
            case HANG:
                yaw.setPosition(RiptideConstants.YAW_HANG);
                pitch.setPosition(RiptideConstants.PITCH_HANG);
                break;
            case BASKET:

                break;
            case SUB:
                yaw.setPosition(RiptideConstants.YAW_SUB);
                pitch.setPosition(RiptideConstants.PITCH_SUB);
                break;
        }
    }
}
