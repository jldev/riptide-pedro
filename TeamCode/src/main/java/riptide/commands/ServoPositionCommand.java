package riptide.commands;

import static java.lang.Math.round;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Servo;

import riptide.subsystems.HorizontalSubsystem;

public class ServoPositionCommand extends CommandBase {

    private final Servo mServo;
    private Double mDesiredPosition;
    private Double mStartingPosition;

    private Boolean mWaitForMove = false;
    private long mMoveEndTime;

    static private double MS_TIME_PER_DEGREE = 140/60; // This is worst case for Axon servos
    static private double SERVO_DEGREES_OF_ROTATION = 355; // we program all ours to max rotation, if this isn't the case this needs to be changed

    public ServoPositionCommand(Servo servo, Double position, Boolean waitForMove) {
        this.mServo = servo;
        this.mDesiredPosition = position;
        this.mStartingPosition = servo.getPosition();
        this.mWaitForMove = waitForMove;
    }

    @Override
    public void initialize() {
       this.mServo.setPosition(mDesiredPosition);
       double moveDelta = Math.abs(mDesiredPosition - mStartingPosition);
       double deltaDegrees = SERVO_DEGREES_OF_ROTATION * moveDelta;
       this.mMoveEndTime = System.currentTimeMillis() + round(deltaDegrees * MS_TIME_PER_DEGREE);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !mWaitForMove || System.currentTimeMillis() > this.mMoveEndTime;
    }
}
