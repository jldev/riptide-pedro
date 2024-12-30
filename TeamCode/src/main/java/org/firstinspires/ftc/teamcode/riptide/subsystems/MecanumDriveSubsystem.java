package org.firstinspires.ftc.teamcode.riptide.subsystems;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class MecanumDriveSubsystem extends SubsystemBase {

    public enum DriveDirection{
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private final MecanumDrive drive;
    private final boolean fieldCentric;

    public MecanumDriveSubsystem(MecanumDrive drive, boolean isFieldCentric) {
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

//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
//        return drive.trajectoryBuilder(startPose);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed, boolean slow) {
//        if (slow){
//            return drive.trajectoryBuilderSlow(startPose, reversed);
//        }
//        return drive.trajectoryBuilder(startPose, reversed);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
//        return drive.trajectoryBuilder(startPose, startHeading);
//    }
//
//    public TrajectorySequenceBuilder trajectorySequenceBuilderSlow(Pose2d startPose){
//        return drive.trajectorySequenceBuilderSlow(startPose);
//    }
//
//    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose){
//        return drive.trajectorySequenceBuilderSlow(startPose);
//    }

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