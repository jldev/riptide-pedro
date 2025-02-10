package org.firstinspires.ftc.teamcode.riptide.subsystems;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.RiptideConstants;

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
        WALL
    }

    private enum GripState {
        OPEN,
        CLOSED
    }
    private GripState mGripState;

    SlideSubsystemState mState;

    public Position slidePosition;

    private final MotorEx mSlideMotor;
    private final PIDFController mSlidePIDController;

    public final Servo shoulder;
    public final Servo elbow;
    public final Servo wrist;
    public final Servo grip;

    private double desiredYaw;
    private boolean deployed;

    public HorizontalSubsystem(Riptide riptide, MotorEx slideMotor1, CommandOpMode opmode, double pos_coefficient, double pos_tolerance, Servo _shoulder, Servo _elbow, Servo _wrist, Servo _grip) {
        mRiptide = riptide;
        mSlideMotor = slideMotor1;
        mOpMode = opmode;

        mSlidePIDController = new PIDFController(RiptideConstants.HORIZONTAL_PID_P, RiptideConstants.HORIZONTAL_PID_I,
                RiptideConstants.HORIZONTAL_PID_D, RiptideConstants.HORIZONTAL_PID_F);
        mSlidePIDController.setTolerance(RiptideConstants.SLIDES_PID_TOLERANCE);
        mSlidePIDController.setSetPoint(0);


        mSlideMotor.stopAndResetEncoder();
        mSlideMotor.setRunMode(MotorEx.RunMode.RawPower);
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.resetEncoder();
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);

        mSlideTargetPosiion = 0;
        slidePosition = Position.HOME;
        mState = SlideSubsystemState.AUTO;

        opmode.telemetry.addLine("Slide Init");
        opmode.telemetry.update();

        shoulder = _shoulder;
        elbow = _elbow;
        wrist = _wrist;
        grip = _grip;

        elbow.setDirection(Servo.Direction.REVERSE);
        mGripState = GripState.OPEN;
        grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);

        deployed = false;
    }


    @Override
    public void periodic() {


        if(mGripState == GripState.CLOSED){ //we can add check for if claw down ect in the future
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_HORIZONTAL);
        } else {
            grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);
        }

        // special case for claw control in sub
        if(slidePosition == Position.SUB){
            if(mRiptide.gunnerOp.getLeftY() < -0.5f)
            {
                shoulder.setPosition(RiptideConstants.HORZ_DEPLOYED_SHOULDER_DOWN);
            } else {
                shoulder.setPosition(RiptideConstants.HORZ_DEPLOYED_SHOULDER);
            }

            //wrist.setPosition(RiptideConstants.HORZ_DEPLOYED_WRIST); // if testing the bellow comment this out

            // idk if this works lol

            if(!deployed) {
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

        if (mState == SlideSubsystemState.AUTO) {
                switch (slidePosition) {
                    case HOME:
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_HOME;
                        deployed = false;
                        break;
                    case OBS:
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_OBS;
                        deployed = false;
                        break;
                    case SUB :
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_SUB;
                        break;
                    case WALL :
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_WALL;
                        deployed = false;
                        break;
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
            if(mSlideTargetPosiion < 0.00)
                mSlideTargetPosiion = 0;
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
                        new InstantCommand(() -> shoulder.setPosition(RiptideConstants.HORZ_HOME_SHOULDER)),
                        new InstantCommand(() -> wrist.setPosition(RiptideConstants.HORZ_HOME_WRIST)),
                        new InstantCommand(() -> elbow.setPosition(RiptideConstants.HORZ_HOME_ELBOW))
                );
            case OBS:
                break;
            case SUB:
                return new SequentialCommandGroup(
                  new InstantCommand(() -> elbow.setPosition(RiptideConstants.HORZ_DEPLOYED_ELBOW))
                );
            case WALL:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> shoulder.setPosition(RiptideConstants.HORZ_WALL_SHOULDER)),
                        new InstantCommand(() -> wrist.setPosition(RiptideConstants.HORZ_WALL_WRIST)),
                        new InstantCommand(() -> elbow.setPosition(RiptideConstants.HORZ_WALL_ELBOW))
                );
        }
        return new WaitCommand(0);
    }

    public void toggleClawState(){
        if (mGripState == GripState.OPEN){
            mGripState = GripState.CLOSED;
        } else {
            mGripState = GripState.OPEN;
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