package org.firstinspires.ftc.teamcode.riptide.subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.riptide.Riptide;

public class HangSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Riptide mRiptide;
    private int mSlideMotorTargetPosition = 0;

    private final CommandOpMode mOpMode;
    private ManualControlDirection mManualDirection;

    public enum ManualControlDirection{
        UP,
        DOWN,
        OFF
    }




    private final MotorEx mSlideMotor;

    public HangSubsystem(Riptide riptide, MotorEx slideMotor, CommandOpMode opmode, double pos_coefficient, double pos_tolerance) {
        mRiptide = riptide;
        mSlideMotor = slideMotor;
        mOpMode = opmode;
        mSlideMotor.stopAndResetEncoder();
        mSlideMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.setPositionCoefficient(pos_coefficient);
        mSlideMotor.setPositionTolerance(pos_tolerance);
        mSlideMotor.setTargetPosition(0);
        mSlideMotorTargetPosition = 0;
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        mSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
        mManualDirection = ManualControlDirection.OFF;
    }


    @Override
    public void periodic(){
        switch (mManualDirection){
            case UP:
                mSlideMotor.set(1);
                break;
            case DOWN:
                mSlideMotor.set(-1);
                break;
            case OFF:
                mSlideMotor.set(0);
                break;
        }
    }





//    public void moveToPosition(SlidesPosition position){
//        // anytime the user wants to move to a position we need to be in auto state
//        changeSlideState(SlideSubsystemState.AUTO);
//        mSlidesCurrentPosition = position;
//    }


    public void stopMotorResetEncoder() {
//        mNeptune.mOpMode.telemetry.addLine("Reset Encoder");
//        mNeptune.mOpMode.telemetry.update();
        mSlideMotor.set(0);
        mSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        mSlideMotor.resetEncoder();
    }
    public void manualSlideControl(ManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        mManualDirection = direction;


    }
    public boolean isBusy (){
        return !mSlideMotor.atTargetPosition();
    }
//    public void addTelemetry(Telemetry telemetry){
//        telemetry.addLine(String.format("current_position - %d", mSlideMotor.getCurrentPosition()));
//        telemetry.addLine(String.format("current_power %.2f", mSlideMotor.motor.getPower()));
//        telemetry.addLine(String.format("target_position %d", mSlideMotorTargetPosition));
//
//
//    }
}