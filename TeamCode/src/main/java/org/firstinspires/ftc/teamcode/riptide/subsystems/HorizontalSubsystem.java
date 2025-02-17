package org.firstinspires.ftc.teamcode.riptide.subsystems;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.RiptideConstants;
import org.firstinspires.ftc.teamcode.riptide.SwitchReader;

public class HorizontalSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Riptide mRiptide;
    private int mSlideTargetPosiion = 0;

    private final CommandOpMode mOpMode;
    private SlideManualControlDirection mSlideManualDirection = SlideManualControlDirection.OFF;

    public enum SlideSubsystemState {
        AUTO,
        MANUAL
    }

    public enum SlideManualControlDirection {
        UP,
        DOWN,
        OFF
    }


    public enum Position {
        HOME,
        OBS,
        SUB,
        HANDSHAKE,
        SPECIFIED
    }

    public enum GripState {
        OPEN,
        CLOSED
    }
    private GripState mGripState;

    private enum DownState {
        UP,
        DOWN
    }
    private DownState mDownState;

    SlideSubsystemState mState;

    public Position slidePosition;

    private Position mServoState;

    public final MotorEx mSlideMotor;
    private final PIDFController mSlidePIDController;

    public final Servo shoulder;
    public final Servo elbow;
    public final Servo wrist;
    public final Servo grip;

    private double desiredYaw;
    private boolean deployed;

    public int specifiedPos = 0;

    public HorizontalSubsystem(Riptide riptide, MotorEx slideMotor1, CommandOpMode opmode, double pos_coefficient, double pos_tolerance, Servo _shoulder, Servo _elbow, Servo _wrist, Servo _grip) {
        mRiptide = riptide;
        mSlideMotor = slideMotor1;
        mOpMode = opmode;

        mSlidePIDController = new PIDFController(RiptideConstants.HORIZONTAL_PID_P, RiptideConstants.HORIZONTAL_PID_I,
                RiptideConstants.HORIZONTAL_PID_D, RiptideConstants.HORIZONTAL_PID_F);
        mSlidePIDController.setTolerance(RiptideConstants.SLIDES_PID_TOLERANCE);
        mSlidePIDController.setSetPoint(0);


        mSlideMotor.stopAndResetEncoder();
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.resetEncoder();
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);

        mSlideTargetPosiion = 0;
        slidePosition = Position.HOME;
        mServoState = Position.HOME;
        mState = SlideSubsystemState.AUTO;

        opmode.telemetry.addLine("Slide Init");
        opmode.telemetry.update();

        shoulder = _shoulder;
        elbow = _elbow;
        wrist = _wrist;
        grip = _grip;

        elbow.setDirection(Servo.Direction.REVERSE);
        shoulder.setDirection(Servo.Direction.REVERSE);

        if(riptide.mOpModeType == Riptide.OpModeType.AUTO) {
            mServoState = Position.HOME;
            shoulder.setPosition(RiptideConstants.HORZ_HOME_SHOULDER);
            elbow.setPosition(RiptideConstants.HORZ_HOME_ELBOW);
            wrist.setPosition(RiptideConstants.HORZ_HOME_WRIST);
            mGripState = GripState.CLOSED;
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_HORIZONTAL);
            mDownState = DownState.UP;
        }

        deployed = false;
    }


    @Override
    public void periodic() {
        mOpMode.telemetry.addData("Servo State", mServoState);
        mOpMode.telemetry.addData("Slide State", slidePosition);
        mOpMode.telemetry.update();

        if(slidePosition == Position.HANDSHAKE && mSlideMotor.getCurrentPosition() < (RiptideConstants.HORIZONTAL_SLIDE_HANDSHAKE + RiptideConstants.SLIDES_PID_TOLERANCE))
        {
            mOpMode.schedule(new SequentialCommandGroup(
                    new WaitCommand(200),
                    mRiptide.GoBasket())
            );
        }


        if(mGripState == GripState.CLOSED){
            if(slidePosition != Position.HANDSHAKE)
            {
                grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_HORIZONTAL);
            } else
            {
                grip.setPosition(RiptideConstants.GRIPPER_CLOSED_HANDSHAKE_VALUE_HORIZONTAL);
            }
        } else {
            grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_HORIZONTAL);
        }

        // special case for claw control in sub
        if(slidePosition == Position.SUB) {

            if (!deployed) {
                wrist.setPosition(RiptideConstants.HORZ_DEPLOYED_WRIST);
                desiredYaw = RiptideConstants.HORZ_DEPLOYED_WRIST;
                deployed = true;
            } else {
                desiredYaw = wrist.getPosition() + mRiptide.gunnerOp.getLeftX() * .045;
                if (desiredYaw > 1.00)
                    desiredYaw = 1.00;
                if (desiredYaw < 0.00)
                    desiredYaw = 0.00;
                wrist.setPosition(desiredYaw);
            }
        }

        if(mServoState == Position.SUB){
            if(mDownState == DownState.DOWN){
                shoulder.setPosition(RiptideConstants.HORZ_DEPLOYED_SHOULDER_DOWN);
            } else {
                shoulder.setPosition(RiptideConstants.HORZ_DEPLOYED_SHOULDER);
            }
        }

        if (mState == SlideSubsystemState.AUTO) {
                switch (slidePosition) {
                    case HOME:
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_HOME;
                        deployed = false;
                        break;
                    case HANDSHAKE:
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_HANDSHAKE;
                    case SPECIFIED:
                        mSlideTargetPosiion = specifiedPos;
                        mOpMode.schedule(changeServos(Position.SUB));
                        mServoState = Position.SUB;
                        if(mSlideMotor.getCurrentPosition() > (specifiedPos - RiptideConstants.SLIDES_PID_TOLERANCE))
                        {
                            toggleClawDownState();
                            toggleClawState();
                            mOpMode.schedule(
                                    new SequentialCommandGroup(
                                            new WaitCommand(1000),
                                            // go to handshake
                                            new InstantCommand(() -> {
                                                slidePosition = Position.HOME;
                                                mServoState = Position.SUB;
                                            })));
                        }
            }
        } else {
            switch (mSlideManualDirection) {
                case UP:
                    mSlideTargetPosiion += RiptideConstants.HORZ_SLIDE_MANUAL_SPEED;
                    mSlidePIDController.setP(RiptideConstants.HORIZONTAL_PID_P);
                    break;
                case DOWN:
                    mSlideTargetPosiion -= RiptideConstants.HORZ_SLIDE_MANUAL_SPEED;
                    mSlidePIDController.setP(RiptideConstants.HORIZONTAL_PID_P);
                    break;
                case OFF:
                    break;
            }
        }

        // add this to stay within the rules of 42" max length
        if(mRiptide.vertical.slidesDeployed()){
            if(mSlideTargetPosiion > RiptideConstants.LENGTH_LIMIT_WHEN_VERTICAL_DEPLOYED){
                mSlideTargetPosiion = RiptideConstants.LENGTH_LIMIT_WHEN_VERTICAL_DEPLOYED;
            }
        }
        mSlidePIDController.setSetPoint(mSlideTargetPosiion);
        double output = mSlidePIDController.calculate(mSlideMotor.getCurrentPosition());
        mSlideMotor.set(output);
    }

    public Command changeServos(Position pos){
        switch(pos){
            case HOME:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> mServoState = Position.HOME),
                        new InstantCommand(() -> shoulder.setPosition(RiptideConstants.HORZ_HOME_SHOULDER)),
                        new InstantCommand(() -> wrist.setPosition(RiptideConstants.HORZ_HOME_WRIST)),
                        new InstantCommand(() -> elbow.setPosition(RiptideConstants.HORZ_HOME_ELBOW)),
                        new InstantCommand(() ->{
                            mGripState = HorizontalSubsystem.GripState.CLOSED;
                            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_HORIZONTAL);
                        })
                );
            case OBS:
                break;
            case SUB:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> shoulder.setPosition(RiptideConstants.HORZ_DEPLOYED_SHOULDER)),
                        new InstantCommand(() -> wrist.setPosition(RiptideConstants.HORZ_DEPLOYED_WRIST)),
                        new InstantCommand(() -> elbow.setPosition(RiptideConstants.HORZ_HOME_ELBOW + .075)),
                        new WaitCommand(200),
                        new InstantCommand(() -> elbow.setPosition(RiptideConstants.HORZ_DEPLOYED_ELBOW)),
                        new InstantCommand(() -> mDownState = DownState.UP),
                        new InstantCommand(() ->{
                            mGripState = GripState.OPEN;
                            grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_HORIZONTAL);
                            mServoState = Position.SUB;
                        })
                );
            case HANDSHAKE:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> mServoState = Position.HANDSHAKE),
                        new InstantCommand(() -> elbow.setPosition(RiptideConstants.HORZ_HANDSHAKE_ELBOW)),
                        new InstantCommand(() -> shoulder.setPosition(RiptideConstants.HORZ_HANDSHAKE_SHOULDER)),
                        new InstantCommand(() -> wrist.setPosition(RiptideConstants.HORZ_HANDSHAKE_WRIST)),
                        new InstantCommand(() ->{
                            mGripState = GripState.CLOSED;
                            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_HANDSHAKE_VALUE_HORIZONTAL);
                        })
                );
        }
        return new WaitCommand(0);
    }

    public void setSpecifiedPos(int pos)
    {
        specifiedPos = pos;
        slidePosition = Position.SPECIFIED;
    }

    public void toggleClawState(){
        if (mGripState == GripState.OPEN){
            mGripState = GripState.CLOSED;
        } else {
            mGripState = GripState.OPEN;
        }
    }

    public void setClawImmediate(HorizontalSubsystem.GripState state){
        mGripState = state;
        if(mGripState == HorizontalSubsystem.GripState.OPEN){
            grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_HORIZONTAL);
        } else {
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_HORIZONTAL);
        }
    }
    public void toggleClawDownState(){
        if (mDownState == DownState.UP){
            mDownState = DownState.DOWN;
        } else {
            mDownState = DownState.UP;
        }
    }

    public void toggleClawAuto(){ //idk if we need this u might have said or not
        if (mGripState == GripState.OPEN){
            mGripState = GripState.CLOSED;
        } else {
            mGripState = GripState.OPEN;
        }

        if(mGripState == GripState.OPEN){
            mDownState = DownState.DOWN;
        } else {
            mDownState = DownState.UP;
        }
    }

    public void changeToSlidePosition(Position pos){
        slidePosition = pos;
        mState = SlideSubsystemState.AUTO;
    }

    public void stopMotorResetEncoder() {
        mSlidePIDController.setSetPoint(0);
        mSlidePIDController.reset();
        mSlideMotor.stopMotor();
        mSlideMotor.resetEncoder();
        mSlideTargetPosiion = 0;
    }

    public void homeSlides(SwitchReader magSwitchButton) {
        int currentPosition = 0;
        while(!magSwitchButton.get()){
            currentPosition = mSlideMotor.getCurrentPosition();
            mSlidePIDController.setSetPoint(currentPosition - 10);
            double output = mSlidePIDController.calculate(
                    mSlideMotor.getCurrentPosition());
            mSlideMotor.set(output);

        }
        stopMotorResetEncoder();
    }

    public void horizontalManualSlideControl(SlideManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        mState = SlideSubsystemState.MANUAL;
        mSlideManualDirection = direction;
    }
    public boolean isBusy (){
        return !mSlidePIDController.atSetPoint();
    }

    public void addTelemetry(Telemetry telemetry){
        mOpMode.telemetry.addData("hCurrent: ", mSlideMotor.encoder.getPosition());
        mOpMode.telemetry.addData("hTarget: ", mSlideTargetPosiion);
    }

    public boolean slidesDeployed(){
        return deployed;
    }
}