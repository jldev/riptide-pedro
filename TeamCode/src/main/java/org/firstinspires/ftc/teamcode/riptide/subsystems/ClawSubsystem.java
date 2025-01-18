package org.firstinspires.ftc.teamcode.riptide.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.RiptideConstants;

public class ClawSubsystem extends SubsystemBase {

    private final Riptide mRiptide;

    private final CommandOpMode mOpMode;

    private final Servo pivot;
    private final Servo shoulder;

    private final Servo elbow;

    private final Servo wrist;
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

    public ClawSubsystem(Riptide riptide, CommandOpMode commandOpMode, Servo _pivot, Servo _shoulder, Servo _elbow, Servo _wrist, Servo _grip) {
        mRiptide = riptide;
        mOpMode = commandOpMode;

        pivot = _pivot;
        shoulder = _shoulder;
        elbow = _elbow;
        wrist = _wrist;
        grip = _grip;

        shoulder.setDirection(Servo.Direction.REVERSE);

        if(riptide.mOpModeType == Riptide.OpModeType.AUTO)
        {
            pivot.setPosition(RiptideConstants.CLAW_PIVOT_INIT);
            shoulder.setPosition(RiptideConstants.CLAW_SHOULDER_INIT);
            elbow.setPosition(RiptideConstants.CLAW_ELBOW_INIT);
            wrist.setPosition(RiptideConstants.CLAW_WRIST_INIT);
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE);
        }
        pivot.setPosition(pivot.getPosition());
        shoulder.setPosition(shoulder.getPosition());
        elbow.setPosition(elbow.getPosition());
        wrist.setPosition(wrist.getPosition());
        grip.setPosition(grip.getPosition());
    }

    @Override
    public void periodic() {

//        desiredYaw = (mHelix.gunnerOp.getLeftX() / 2) + .5;
//        desiredPitch = (mHelix.gunnerOp.getLeftY() / 2) + .5;





        //   GUNNER CONTROL


        desiredYaw = pivot.getPosition();
        desiredPitch = shoulder.getPosition();

        if (Math.abs(mRiptide.gunnerOp.getLeftX()) > 0.1 || Math.abs(mRiptide.gunnerOp.getLeftY()) > 0.1) {
            desiredYaw = pivot.getPosition() + mRiptide.gunnerOp.getLeftX() * .045;
            desiredPitch = shoulder.getPosition() + mRiptide.gunnerOp.getLeftY() * .045;
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


//        pivot.setPosition(desiredYaw);
//        shoulder.setPosition(desiredPitch);
          pivot.setPosition(RiptideConstants.HORZ_DEPLOYED_PIVOT);
          shoulder.setPosition(RiptideConstants.HORZ_DEPLOYED_SHOULDER);
          elbow.setPosition(RiptideConstants.HORZ_DEPLOYED_ELBOW);
          wrist.setPosition(RiptideConstants.HORZ_DEPLOYED_WRIST);


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
                pivot.setPosition(RiptideConstants.YAW_HOME);
                shoulder.setPosition(RiptideConstants.PITCH_HOME);
                break;
            case HANG:
                pivot.setPosition(RiptideConstants.YAW_HANG);
                shoulder.setPosition(RiptideConstants.PITCH_HANG);
                break;
            case BASKET:

                break;
            case SUB:
                pivot.setPosition(RiptideConstants.YAW_SUB);
                shoulder.setPosition(RiptideConstants.PITCH_SUB);
                break;
        }
    }
}
