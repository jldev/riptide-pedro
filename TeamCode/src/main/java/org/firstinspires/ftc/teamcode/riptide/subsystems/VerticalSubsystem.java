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

public class VerticalSubsystem extends SubsystemBase {
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
        WALL,
        HANG,
        BASKET,
        PRELOAD_BASKET,
        ENDGAME
    }



    SlideSubsystemState mState;

    public Position slidePosition;
    public Position prevSlidePosition;

    private final MotorEx mSlideMotor1;
    private final MotorEx mSlideMotor2;
    private final PIDFController mSlide1PIDController;
    private final PIDFController mSlide2PIDController;

    public final Servo shoulder1;
    public final Servo shoulder2;
    public final Servo rotation;
    public final Servo elbow;
    public final Servo grip;
    public double defaultState;

    public VerticalSubsystem(Riptide riptide, MotorEx slideMotor1, MotorEx slideMotor2, CommandOpMode opmode, double pos_coefficient, double pos_tolerance, Servo _shoulder1, Servo _shoulder2, Servo _rotation, Servo _elbow, Servo _grip) {
        mRiptide = riptide;
        mSlideMotor1 = slideMotor1;
        mSlideMotor2 = slideMotor2;
        mOpMode = opmode;

        mSlide1PIDController = new PIDFController(RiptideConstants.VERTICAL_PID_P, RiptideConstants.VERTICAL_PID_I,
                RiptideConstants.VERTICAL_PID_D, RiptideConstants.VERTICAL_PID_F);
        mSlide1PIDController.setTolerance(RiptideConstants.SLIDES_PID_TOLERANCE);
        mSlide1PIDController.setSetPoint(0);

        mSlide2PIDController = new PIDFController(RiptideConstants.VERTICAL_PID_P, RiptideConstants.VERTICAL_PID_I,
                RiptideConstants.VERTICAL_PID_D, RiptideConstants.VERTICAL_PID_F);
        mSlide2PIDController.setTolerance(RiptideConstants.SLIDES_PID_TOLERANCE);
        mSlide2PIDController.setSetPoint(0);


        mSlideMotor1.stopAndResetEncoder();
        mSlideMotor1.setRunMode(MotorEx.RunMode.RawPower);
        mSlideMotor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor1.resetEncoder();
        mSlideMotor1.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        mSlideMotor1.encoder.setDirection(Motor.Direction.FORWARD);

        mSlideMotor2.stopAndResetEncoder();
        mSlideMotor2.setRunMode(MotorEx.RunMode.RawPower);
        mSlideMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor2.resetEncoder();
        mSlideMotor2.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor2.encoder.setDirection(Motor.Direction.FORWARD);


        shoulder1 = _shoulder1;
        shoulder2 = _shoulder2;
        rotation = _rotation;
        elbow = _elbow;
        grip = _grip;

        shoulder1.setDirection(Servo.Direction.REVERSE);



        mSlideTargetPosiion = 0;
        slidePosition = Position.HOME;
        prevSlidePosition = Position.HANG;
//        grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);
        mState = SlideSubsystemState.AUTO;

        shoulder1.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
        shoulder2.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
        elbow.setPosition(RiptideConstants.VERT_HOME_ELBOW);
        defaultState = RiptideConstants.GRIPPER_OPEN_VALUE;
        grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);

        opmode.telemetry.addLine("Slide Init");
        opmode.telemetry.update();
    }


    @Override
    public void periodic() {

        if(mRiptide.gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            if(defaultState == RiptideConstants.GRIPPER_OPEN_VALUE) {
                grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
            } else {
                grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);
            }
        } else{
            grip.setPosition(defaultState);
        }

        if (mState == SlideSubsystemState.AUTO) {
                    switch (slidePosition) {
                        case HOME:
                            mSlideTargetPosiion = RiptideConstants.VERTICAL_SLIDE_HOME;
                            break;
                        case WALL:
                            mSlideTargetPosiion = RiptideConstants.VERTICAL_SLIDE_WALL;
                            break;
                        case HANG:
                            mSlideTargetPosiion = RiptideConstants.VERTICAL_SLIDE_HANG;
                            break;
                        case BASKET:
                            mSlideTargetPosiion = RiptideConstants.VERTICAL_SLIDE_BASKET;
                            break;
                        case PRELOAD_BASKET:
                            mSlideTargetPosiion = RiptideConstants.VERTICAL_PRELOAD_BASKET;
                            break;
                        case ENDGAME:
                            mSlideTargetPosiion = RiptideConstants.VERTICAL_ENDGAME;
                            break;
                    }
        } else {
            switch (mSlideManualDirection) {
                case UP:
                    mSlideTargetPosiion += RiptideConstants.VERT_SLIDE_MANUAL_SPEED;
                    mSlide1PIDController.setP(RiptideConstants.VERTICAL_PID_P);
                    mSlide2PIDController.setP(RiptideConstants.VERTICAL_PID_P);
                    break;
                case DOWN:
                    mSlideTargetPosiion -= RiptideConstants.VERT_SLIDE_MANUAL_SPEED;
                    mSlide1PIDController.setP(0.015);
                    mSlide2PIDController.setP(0.015);
                    break;
                case OFF:
                    break;
            }
            if(mSlideTargetPosiion < 0.00)
                mSlideTargetPosiion = 0;
        }

        mSlide1PIDController.setSetPoint(mSlideTargetPosiion);
        double output = mSlide1PIDController.calculate(
                mSlideMotor1.getCurrentPosition());
            mSlideMotor1.set(output);

        mSlide2PIDController.setSetPoint(mSlideTargetPosiion);
        output = mSlide2PIDController.calculate(
                mSlideMotor2.getCurrentPosition());
        mSlideMotor2.set(output);
    }

    public Command changeServos(Position pos) {
                switch (pos) {
                    case HOME:
                        return new SequentialCommandGroup(
                                new InstantCommand(() -> rotation.setPosition(RiptideConstants.VERT_HOME_ROTATION)),
                                new WaitCommand(750),
                                new InstantCommand(()-> {
                                    shoulder1.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
                                    shoulder2.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
                                    elbow.setPosition(RiptideConstants.VERT_HOME_ELBOW);
                                    defaultState = RiptideConstants.GRIPPER_OPEN_VALUE;
                                    grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE);
                                }));
                    case WALL:
                        return new SequentialCommandGroup(
                                new InstantCommand(() -> defaultState = RiptideConstants.GRIPPER_OPEN_VALUE),
                                new InstantCommand(() -> grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE)),
                                new InstantCommand(() -> rotation.setPosition(RiptideConstants.VERT_WALL_ROTATION)),
                                new WaitCommand(750),
                    new InstantCommand(()-> {
                        shoulder1.setPosition(RiptideConstants.VERT_WALL_SHOULDER);
                        shoulder2.setPosition(RiptideConstants.VERT_WALL_SHOULDER);
                        elbow.setPosition(RiptideConstants.VERT_WALL_ELBOW);
                    }));
                    case HANG:
                        return new SequentialCommandGroup(
                                new InstantCommand(()-> {
                                    defaultState = RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL;
                                    grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
                                }),
                                new InstantCommand(()-> {
                                    shoulder1.setPosition(RiptideConstants.VERT_HANG_SHOULDER);
                                    shoulder2.setPosition(RiptideConstants.VERT_HANG_SHOULDER);
                                    elbow.setPosition(RiptideConstants.VERT_HANG_ELBOW);
                                }),
                                new WaitCommand(750),
                                new InstantCommand(() -> rotation.setPosition(RiptideConstants.VERT_HANG_ROTATION))
                        );
                    case BASKET:
                        return new SequentialCommandGroup(
                                new InstantCommand(()-> {
                                    defaultState = RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL;
                                    grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
                                }),
                                new InstantCommand(()-> {
                                    shoulder1.setPosition(RiptideConstants.VERT_BASKET_SHOULDER);
                                    shoulder2.setPosition(RiptideConstants.VERT_BASKET_SHOULDER);
                                    elbow.setPosition(RiptideConstants.VERT_BASKET_ELBOW);
                                }),
                                new WaitCommand(750),
                                new InstantCommand(() -> rotation.setPosition(RiptideConstants.VERT_BASKET_ROTATION))
                        );
                    case PRELOAD_BASKET:
                    case ENDGAME:
                }
                return new WaitCommand(0);
    }

    private void changeSlideState(SlideSubsystemState newState){
        mState = newState;
    }


    public void changePositionTo(Position pos){
        slidePosition = pos;
        changeSlideState(SlideSubsystemState.AUTO);
    }





//    public void moveToPosition(SlidesPosition position){
//        // anytime the user wants to move to a position we need to be in auto state
//        changeSlideState(SlideSubsystemState.AUTO);
//        mSlidesCurrentPosition = position;
//    }


    public void stopMotorResetEncoder() {
//        mNeptune.mOpMode.telemetry.addLine("Reset Encoder");
//        mNeptune.mOpMode.telemetry.update();
        mSlide1PIDController.setSetPoint(0);
        mSlide1PIDController.reset();
        mSlideMotor1.stopMotor();
        mSlideMotor1.resetEncoder();
        mSlide2PIDController.setSetPoint(0);
        mSlide2PIDController.reset();
        mSlideMotor2.stopMotor();
        mSlideMotor2.resetEncoder();
//        mHorizontalPIDController.clearTotalError();
//        mHorizontalPIDController.setSetPoint(0);
//        mHorizontalSlideMotor.stopMotor();
//        mHorizontalSlideMotor.resetEncoder();
    }




    public void verticalManualSlideControl(SlideManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        mSlideManualDirection = direction;
    }


    public boolean isBusy (){
        return !mSlide1PIDController.atSetPoint();
    }

    public void addTelemetry(Telemetry telemetry){
        mOpMode.telemetry.addData("vCurrent1: ", mSlideMotor1.encoder.getPosition());
        mOpMode.telemetry.addData("vCurrent2: ", mSlideMotor2.encoder.getPosition());
        mOpMode.telemetry.addData("vTarget: ", mSlideTargetPosiion);
    }
}