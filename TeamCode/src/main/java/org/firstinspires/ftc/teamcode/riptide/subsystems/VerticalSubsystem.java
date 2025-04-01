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

public class VerticalSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Riptide mRiptide;
    private int mSlideTargetPosiion = 0;
//    private int mDesiredPosition = 0;
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

    public enum SlideSpeed {
        SLOW,
        HIGH

    }

    public enum Position {
        HOME,
        WALL,
        HANG,
        BASKET,
        PRELOAD_BASKET,
        ENDGAME,
        HANDSHAKE
    }

    public enum GripState {
        OPEN,
        CLOSED
    }

    public GripState mGripState;
    SlideSubsystemState mState;

    public Position slidePosition;
    public Position prevSlidePosition;

    private final MotorEx mSlideMotor1;
    private final MotorEx mSlideMotor2;
    private final PIDFController mSlidePIDController;

    public final Servo shoulder1;
    public final Servo shoulder2;
    public final Servo rotation;
    public final Servo elbow;
    public final Servo grip;
    public final Servo speedSwitch;

    public VerticalSubsystem(Riptide riptide, MotorEx slideMotor1, MotorEx slideMotor2, CommandOpMode opmode, double pos_coefficient, double pos_tolerance, Servo _shoulder1, Servo _shoulder2, Servo _rotation, Servo _elbow, Servo _grip, Servo _speedSwitch) {
        mRiptide = riptide;
        mSlideMotor1 = slideMotor1;
        mSlideMotor2 = slideMotor2;
        mOpMode = opmode;

        mSlidePIDController = new PIDFController(RiptideConstants.VERTICAL_PID_P, RiptideConstants.VERTICAL_PID_I,
                RiptideConstants.VERTICAL_PID_D, RiptideConstants.VERTICAL_PID_F);
        mSlidePIDController.setTolerance(RiptideConstants.SLIDES_PID_TOLERANCE);
        mSlidePIDController.setSetPoint(mSlideMotor1.getCurrentPosition());


        if(riptide.mOpModeType == Riptide.OpModeType.AUTO) {
            mSlideMotor1.resetEncoder();
            mSlideMotor2.resetEncoder();
            slidePosition = Position.HOME;
            prevSlidePosition = Position.HANG;
        } else {
//            slidePosition = Position.HANG;
            slidePosition = Position.HOME;
            prevSlidePosition = Position.HOME;
        }

        mSlideMotor1.stopAndResetEncoder();
        mSlideMotor1.setRunMode(MotorEx.RunMode.RawPower);
        mSlideMotor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mSlideMotor1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor1.encoder.setDirection(Motor.Direction.FORWARD);

        mSlideMotor2.stopAndResetEncoder();
        mSlideMotor2.setRunMode(MotorEx.RunMode.RawPower);
        mSlideMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mSlideMotor2.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        mSlideMotor2.encoder.setDirection(Motor.Direction.FORWARD);


        shoulder1 = _shoulder1;
        shoulder2 = _shoulder2;
        rotation = _rotation;
        elbow = _elbow;
        grip = _grip;
        speedSwitch = _speedSwitch;

        shoulder2.setDirection(Servo.Direction.REVERSE);

        mSlideTargetPosiion = mSlideMotor1.getCurrentPosition();
//        mDesiredPosition = mSlideMotor1.getCurrentPosition();

        mState = SlideSubsystemState.AUTO;

        if(riptide.mOpModeType == Riptide.OpModeType.AUTO) {
            shoulder1.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
            shoulder2.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
            rotation.setPosition(RiptideConstants.VERT_HOME_ROTATION);
            elbow.setPosition(RiptideConstants.VERT_HOME_ELBOW);
            mGripState = GripState.CLOSED;
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
        }

        opmode.telemetry.addLine("Slide Init");
        opmode.telemetry.update();
    }

    @Override
    public void periodic() {
        // for testing
//        shoulder1.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
//        shoulder2.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
//        rotation.setPosition(RiptideConstants.VERT_HOME_ROTATION);
//        elbow.setPosition(RiptideConstants.VERT_HOME_ELBOW);
//        mGripState = GripState.CLOSED;
//        grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
        //


        if(mGripState == GripState.CLOSED){ //we can add check for if claw down ect in the future
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
        } else {
            grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_VERTICAL);
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
            }
        } else {
            switch (mSlideManualDirection) {
                case UP:
                    mSlideTargetPosiion += RiptideConstants.VERT_SLIDE_MANUAL_SPEED;
                    break;
                case DOWN:
                    mSlideTargetPosiion -= RiptideConstants.VERT_SLIDE_MANUAL_SPEED;
                    break;
                case OFF:
                    break;
            }

            if(mRiptide.magSwitchButton1.get()){
                if(mSlideTargetPosiion < 0){
                    mSlideTargetPosiion = 0;
                }
            }

            if(mSlideTargetPosiion > RiptideConstants.VERTICAL_SLIDE_BASKET)
                mSlideTargetPosiion = RiptideConstants.VERTICAL_SLIDE_BASKET;

            // add this to stay within the rules of 42" max length
            if(mRiptide.horizontal.getCurrentPosition() > RiptideConstants.LENGTH_LIMIT_WHEN_VERTICAL_DEPLOYED){
                if (mSlideTargetPosiion > RiptideConstants.HEIGHT_LIMIT_WHEN_HORIZONTAL_DEPLOYED){
                    mSlideTargetPosiion = RiptideConstants.HEIGHT_LIMIT_WHEN_HORIZONTAL_DEPLOYED;
                }
            }
        }


        mSlidePIDController.setSetPoint(mSlideTargetPosiion);

        if(mSlidePIDController.atSetPoint()){
            mSlidePIDController.setSetPoint(mSlideMotor1.getCurrentPosition());
        }

        double output1 = mSlidePIDController.calculate(mSlideMotor1.getCurrentPosition());
        mSlideMotor1.set(output1);
        mSlideMotor2.set(output1);
    }

    public Command changeServos(Position pos) {
                switch (pos) {
                    case HOME: //this might be our init
                        return new SequentialCommandGroup(
                                new InstantCommand(() -> rotation.setPosition(RiptideConstants.VERT_HOME_ROTATION)),
                                new WaitCommand(750),
                                new InstantCommand(()-> {
                                    shoulder1.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
                                    shoulder2.setPosition(RiptideConstants.VERT_HOME_SHOULDER);
                                    //i apologize for this if statement if too tired to care rn
                                    if(mRiptide.horizontal.slidePosition == HorizontalSubsystem.Position.SUB)
                                    {
                                        elbow.setPosition(1);
                                    } else
                                    {
                                        elbow.setPosition(RiptideConstants.VERT_HOME_ELBOW);
                                    }
                                    mGripState = GripState.OPEN;
                                    grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_VERTICAL);
                                }));
                    case WALL:
                        return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    mGripState = GripState.OPEN;
                                    grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_VERTICAL);
                                    rotation.setPosition(RiptideConstants.VERT_WALL_ROTATION);
                                }),
                                new WaitCommand(750),
                                new InstantCommand(()-> {
                                    shoulder1.setPosition(RiptideConstants.VERT_WALL_SHOULDER);
                                    shoulder2.setPosition(RiptideConstants.VERT_WALL_SHOULDER);
                                    elbow.setPosition(RiptideConstants.VERT_WALL_ELBOW);
                                }));
                    case HANG:
                        return new SequentialCommandGroup(
                                new InstantCommand(()-> {
                                    mGripState = GripState.CLOSED;
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
                                    mGripState = GripState.CLOSED;
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

                    case HANDSHAKE:
                        return new SequentialCommandGroup(
                                new InstantCommand(() -> rotation.setPosition(RiptideConstants.VERT_HOME_ROTATION)),
                                new WaitCommand(750),
                                new InstantCommand(()-> {
                                    shoulder1.setPosition(RiptideConstants.VERT_HANDSHAKE_SHOULDER);
                                    shoulder2.setPosition(RiptideConstants.VERT_HANDSHAKE_SHOULDER);
                                    elbow.setPosition(RiptideConstants.VERT_HANDSHAKE_ELBOW);
                                    mGripState = GripState.OPEN;
                                    grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_VERTICAL);
                                }));

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

    public void toggleClawState(){
        if (mGripState == GripState.OPEN){
            mGripState = GripState.CLOSED;
        } else {
            mGripState = GripState.OPEN;
        }
    }

    public void setClawImmediate(GripState state){
        mGripState = state;
        if(mGripState == GripState.OPEN){
            grip.setPosition(RiptideConstants.GRIPPER_OPEN_VALUE_VERTICAL);
        } else {
            grip.setPosition(RiptideConstants.GRIPPER_CLOSED_VALUE_VERTICAL);
        }
    }

    public void stopMotorResetEncoder() {
        mSlideMotor1.stopMotor();
        mSlideMotor2.stopMotor();

        mSlidePIDController.setSetPoint(0);
        mSlideTargetPosiion = 0;
        mSlidePIDController.reset();
        mSlideMotor1.resetEncoder();

        mSlideTargetPosiion = 0;
        mSlideMotor2.resetEncoder();
    }
    public void verticalManualSlideControl(SlideManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        mSlideManualDirection = direction;
    }
    public void toggleMotorSpeed(){
        if(speedSwitch.getPosition() == RiptideConstants.SPEED_SERVO_FAST) {
            speedSwitch.setPosition(RiptideConstants.SPEED_SERVO_SLOW);
        }else{
            speedSwitch.setPosition(RiptideConstants.SPEED_SERVO_FAST);
        }
    }

    public boolean slidesDeployed(){
        return mSlideTargetPosiion > RiptideConstants.VERTICAL_SLIDE_HANG;
    }
    public boolean isBusy (){
        return !mSlidePIDController.atSetPoint();
    }

    public void addTelemetry(Telemetry telemetry){
        mOpMode.telemetry.addData("vCurrent1: ", mSlideMotor1.encoder.getPosition());
        mOpMode.telemetry.addData("vCurrent2: ", mSlideMotor2.encoder.getPosition());
        mOpMode.telemetry.addData("vTarget: ", mSlideTargetPosiion);
    }
}