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

public class PivotSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Riptide mRiptide;
    private int mTargetPosiion = 0;

    private final CommandOpMode mOpMode;
    private ManualControlDirection mManualDirection = ManualControlDirection.OFF;

    public enum SlideSubsystemState {
        AUTO,
        MANUAL
    }

    public enum ManualControlDirection {
        UP,
        DOWN,
        OFF
    }


    public enum SlidePosition{
        HOME,
        HANG,
        BASKET,
        SUB,
        PRELOAD_BASKET
    }



    SlideSubsystemState mState;

    public SlidePosition position;

    private final MotorEx mVerticalSlideMotor;
    private final PIDFController mVerticalPIDController;

    public PivotSubsystem(Riptide riptide, MotorEx verticalSlideMotor, CommandOpMode opmode, double pos_coefficient, double pos_tolerance) {
        mRiptide = riptide;
        mVerticalSlideMotor = verticalSlideMotor;
        mOpMode = opmode;
        mVerticalPIDController = new PIDFController(RiptideConstants.PIVOT_PID_P, RiptideConstants.PIVOT_PID_I,
                RiptideConstants.PIVOT_PID_D, RiptideConstants.PIVOT_PID_F);
        mVerticalPIDController.setTolerance(RiptideConstants.SLIDES_PID_TOLERANCE);
        ;
        mVerticalSlideMotor.stopAndResetEncoder();
        mVerticalSlideMotor.setRunMode(MotorEx.RunMode.RawPower);
        mVerticalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mVerticalPIDController.setSetPoint(0);
        position = SlidePosition.HOME;
        mTargetPosiion = 0;
        mVerticalSlideMotor.resetEncoder();
        mVerticalSlideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        mVerticalSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
        mState = SlideSubsystemState.AUTO;
        opmode.telemetry.addLine("Slide Init");
        opmode.telemetry.update();
    }


    @Override
    public void periodic() {
        if (mState == SlideSubsystemState.AUTO) {

                switch (position) {
                    case HOME:
                        mTargetPosiion = RiptideConstants.PIVOT_HOME;
                        break;
                    case HANG:
                        mTargetPosiion = RiptideConstants.PIVOT_HANG;
                        break;
                    case BASKET:
                        mTargetPosiion = RiptideConstants.PIVOT_BASKET;
                        break;
                    case SUB:
                        mTargetPosiion = RiptideConstants.PIVOT_SUB;
                        break;
                    case PRELOAD_BASKET:
                        mTargetPosiion = RiptideConstants.PIVOT_PRELOAD_BASKET;
                        break;
            }
        } else {
            switch (mManualDirection) {
                case UP:
                    mTargetPosiion += RiptideConstants.PIVOT_MANUAL_SPEED;
                    break;
                case DOWN:
                    mTargetPosiion -= RiptideConstants.PIVOT_MANUAL_SPEED;
                    break;
                case OFF:
                    break;
            }
        }

        mVerticalPIDController.setSetPoint(mTargetPosiion);
        double output = mVerticalPIDController.calculate(
                mVerticalSlideMotor.getCurrentPosition());
            mVerticalSlideMotor.set(output);



    }

    private void changeSlideState(SlideSubsystemState newState){
        mState = newState;
    }


    public void changeToSlidePosition(SlidePosition pos){
        position = pos;
        changeSlideState(SlideSubsystemState.AUTO);
    }





//    public void moveToPosition(SlidesPosition position){
//        // anytime the user wants to move to a position we need to be in auto state
//        changeSlideState(SlideSubsystemState.AUTO);
//        mSlidesCurrentPosition = position;
//    }


    public void stopMotorResetEncoder() {
        mVerticalPIDController.setSetPoint(0);
        mVerticalPIDController.reset();
        mVerticalSlideMotor.stopMotor();
        mVerticalSlideMotor.resetEncoder();
    }




    public void ManualPivotControl(ManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        mManualDirection = direction;
    }


    public boolean isBusy (){
        return !mVerticalPIDController.atSetPoint();
    }

    public void addTelemetry(Telemetry telemetry){
        mOpMode.telemetry.addData("pCurrent: ", mVerticalSlideMotor.encoder.getPosition());
        mOpMode.telemetry.addData("pTarget: ", mTargetPosiion);
    }
}