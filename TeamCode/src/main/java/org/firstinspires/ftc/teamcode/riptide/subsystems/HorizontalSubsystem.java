package org.firstinspires.ftc.teamcode.riptide.subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        OBS
    }



    SlideSubsystemState mState;

    public Position slidePosition;

    private final MotorEx mSlideMotor;
    private final PIDFController mSlidePIDController;

    public HorizontalSubsystem(Riptide riptide, MotorEx slideMotor1, CommandOpMode opmode, double pos_coefficient, double pos_tolerance) {
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
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        mSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);



        mSlideTargetPosiion = 0;
        slidePosition = Position.HOME;
        mState = SlideSubsystemState.AUTO;
        opmode.telemetry.addLine("Slide Init");
        opmode.telemetry.update();
    }


    @Override
    public void periodic() {
        if (mState == SlideSubsystemState.AUTO) {

                switch (slidePosition) {
                    case HOME:
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_HOME;
                        break;
                    case OBS:
                        mSlideTargetPosiion = RiptideConstants.HORIZONTAL_SLIDE_OBS;
                        break;
            }
        } else {
            switch (mSlideManualDirection) {
                case UP:
                    mSlideTargetPosiion += RiptideConstants.SLIDE_MANUAL_SPEED;
                    mSlidePIDController.setP(RiptideConstants.HORIZONTAL_PID_P);
                    break;
                case DOWN:
                    mSlideTargetPosiion -= RiptideConstants.SLIDE_MANUAL_SPEED;
                    mSlidePIDController.setP(0.015);
                    break;
                case OFF:
                    break;
            }
            if(mSlideTargetPosiion < 0.00)
                mSlideTargetPosiion = 0;
        }

        mSlidePIDController.setSetPoint(mSlideTargetPosiion);
        double output = mSlidePIDController.calculate(
                mSlideMotor.getCurrentPosition());
            mSlideMotor.set(output);
    }

    private void changeSlideState(SlideSubsystemState newState){
        mState = newState;
    }


    public void changeToSlidePosition(Position pos){
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
        mSlidePIDController.setSetPoint(0);
        mSlidePIDController.reset();
        mSlideMotor.stopMotor();
        mSlideMotor.resetEncoder();

//        mHorizontalPIDController.clearTotalError();
//        mHorizontalPIDController.setSetPoint(0);
//        mHorizontalSlideMotor.stopMotor();
//        mHorizontalSlideMotor.resetEncoder();
    }




    public void horizontalManualSlideControl(SlideManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        mSlideManualDirection = direction;
    }


    public boolean isBusy (){
        return !mSlidePIDController.atSetPoint();
    }

    public void addTelemetry(Telemetry telemetry){
        mOpMode.telemetry.addData("hCurrent: ", mSlideMotor.encoder.getPosition());
        mOpMode.telemetry.addData("hTarget: ", mSlideTargetPosiion);
    }
}