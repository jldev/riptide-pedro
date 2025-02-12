package org.firstinspires.ftc.teamcode.riptide.subsystems;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class NewDrive extends SubsystemBase {

    public enum DriveDirection{
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private final MecanumDrive drive;
    private final boolean fieldCentric;

    public NewDrive(MecanumDrive drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.pose = pose;
    }

    @Override
    public void periodic(){
        drive.updatePoseEstimate();
    }
    public void drive(double leftY, double leftX, double rightX) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        leftY,
                        leftX
                ),
                rightX
        ));
    }



    // dont even ask what this is

//    public void moveTo(double x, double y) {
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose).splineTo(new Vector2d(x, y), new Rotation2d(0, 0)).build()
//        );
//    }


    // i doubt this will go anywhere
        public void moveTo(double x, double y) {
        Actions.runBlocking( new ParallelAction(
                drive.actionBuilder(drive.pose).lineToX(x).build(),
                drive.actionBuilder(drive.pose).lineToY(y).build()
                )
        );
    }

    public void driveDirection(DriveDirection direction, double inches){
        switch (direction){

            case FORWARD:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose).lineToX(drive.pose.position.x + inches).build()
                );
                break;
            case BACKWARD:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose).lineToX(drive.pose.position.x - inches).build()
                );
                break;
            case LEFT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose).lineToY(drive.pose.position.y + inches).build()
                );
                break;
            case RIGHT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose).lineToY(drive.pose.position.y - inches).build()
                );
                break;
        }
    }
    public Pose2d getPoseEstimate() {
        return drive.pose;
    }


    public void followAction(Action action) {
        Actions.runBlocking(action);
    }

    public boolean isBusy() {
        return (drive.leftBack.isBusy() ||
                drive.leftFront.isBusy() ||
                drive.rightBack.isBusy() ||
                drive.rightFront.isBusy());
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
    }
}