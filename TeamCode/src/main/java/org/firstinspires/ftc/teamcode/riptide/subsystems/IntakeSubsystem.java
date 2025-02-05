package org.firstinspires.ftc.teamcode.riptide.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.RiptideConstants;

public class IntakeSubsystem extends SubsystemBase {


    private final Servo mLiftServo;
    private final Servo mGripperServo;
    private double mIntakeLiftPosition = 0.0;
    private boolean intakeAutoControl = false;

    private Riptide mRiptide;

    public IntakeSubsystem(Riptide riptide, Servo liftServo, Servo gripperServo) {
        mLiftServo = liftServo;
        mGripperServo = gripperServo;
        mLiftServo.setDirection(Servo.Direction.REVERSE);
        mIntakeLiftPosition = RiptideConstants.LIFT_POS_0;
        mLiftServo.setPosition(mIntakeLiftPosition);
        mGripperServo.setPosition(mIntakeLiftPosition);
        mRiptide = riptide;

        if (riptide.mOpModeType == Riptide.OpModeType.AUTO) {
            intakeAutoControl = true;
        }


    }


    public enum GripperState {
        CLOSED,

        OPEN,
    }

    public enum LiftState {
        P0,
        P1,
        P2,
    }


    GripperState gripperState = GripperState.CLOSED;
    LiftState liftState = LiftState.P0;

    @Override
    public void periodic() {


        switch (gripperState) {
            case OPEN:
                mGripperServo.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);
                break;
            case CLOSED:
                mGripperServo.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
                break;
        }

    }

    public boolean intakeFull() {
        return false;
    }


    public InstantCommand setGripperOpen() {
        return new InstantCommand(() -> gripperState = GripperState.OPEN);
    }

    public InstantCommand setGripperClosed() {
        return new InstantCommand(() -> gripperState = GripperState.CLOSED);
    }

    public void cycleLift() {
        mRiptide.mOpMode.telemetry.addLine("put a string in there " + mRiptide.mOpMode.getRuntime());
        mRiptide.mOpMode.telemetry.update();

        if (liftState == LiftState.P0) {
            liftState = LiftState.P1;
            mLiftServo.setPosition(RiptideConstants.LIFT_POS_1);
        }
        else if (liftState == LiftState.P1) {
            liftState = LiftState.P2;
            mLiftServo.setPosition(RiptideConstants.LIFT_POS_2);
        }
        else if (liftState == LiftState.P2) {
            liftState = LiftState.P0;
            mLiftServo.setPosition(RiptideConstants.LIFT_POS_0);
        }
    }

    // temp logic
    public void setLift(double position)
    {
        mLiftServo.setPosition(RiptideConstants.LIFT_POS_1);
    }
}
