package org.firstinspires.ftc.teamcode.riptide.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.commands.RoadRunnerDrive;

public class RiptideAuto {

    public Riptide riptide;

    private CommandOpMode opMode;
    Task currentState = Task.PRELOAD_DRIVE;

    public Pose2d desiredPosition;

    private enum Task{
        // specimen
        PRELOAD_DRIVE,
        PUSH_SAMPLES,
        HANG_SPECIMEN,
        RETRIEVE_SPECIMEN,
        PARK,

        // basket
        PRELOAD_BASKET_DRIVE,
        RETRIEVE_SAMPLE,
        DEPOSIT_SAMPLE,
        PARK_BASKET,

        // also this
        WAIT_FOR_TASK
    }

    private int runCount = 0;

    public RiptideAuto(CommandOpMode commandOpMode, Riptide.FieldPos startingPosition, Riptide.AllianceColor allianceColor, Riptide.Target target) {
        opMode = commandOpMode;
        riptide = new Riptide(opMode, Riptide.OpModeType.AUTO, allianceColor);
        riptide.setStartPosition(startingPosition, allianceColor);
        riptide.target = target;
        if(riptide.target == Riptide.Target.SPECIMENS)
        {
            currentState = Task.PRELOAD_DRIVE;
        } else
        {
            currentState = Task.PRELOAD_BASKET_DRIVE;
        }
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(180));
        riptide.setStartPosition(startPos);
    }

    public void run() {
        opMode.telemetry.addData("Current State", currentState);
        opMode.telemetry.addData("Run Count", runCount++);
        opMode.telemetry.addLine(String.format("Pose X: %.2f, Y: %.2f, Rot: %.2f", riptide.drive.getPoseEstimate().position.x,
                riptide.drive.getPoseEstimate().position.y, Math.toDegrees(riptide.drive.getPoseEstimate().heading.toDouble())));
        opMode.telemetry.update();

        switch (currentState) {

            // specimen

            case PRELOAD_DRIVE:
                opMode.schedule(
                        new SequentialCommandGroup(
                                riptide.GoHang(),
                                new RoadRunnerDrive(32, 0, riptide.drive),
                                new InstantCommand(() -> riptide.vertical.toggleClawState()),
                                new RoadRunnerDrive(-6, 0, riptide.drive),
                                riptide.GoWall(),
                                new InstantCommand(() -> currentState = Task.PUSH_SAMPLES)
                            )
                        );
                currentState = Task.WAIT_FOR_TASK;
                break;
            case PUSH_SAMPLES:
                opMode.schedule(
                        new SequentialCommandGroup(
                                riptide.GoWall(),
                                new RoadRunnerDrive(0, -32, riptide.drive),
                                new RoadRunnerDrive(26, 0, riptide.drive),
                                new RoadRunnerDrive(0, -8, riptide.drive),
                                new RoadRunnerDrive(-44, 0, riptide.drive),
                                new RoadRunnerDrive(44, 0, riptide.drive),
                                new RoadRunnerDrive(0, -12, riptide.drive),
                                new RoadRunnerDrive(-46, 0, riptide.drive),
                                new RoadRunnerDrive(-10, 0, riptide.drive),
                                new InstantCommand(() -> currentState = Task.HANG_SPECIMEN))
                );
                currentState = Task.WAIT_FOR_TASK;
                break;
            case HANG_SPECIMEN:
                opMode.schedule(new SequentialCommandGroup(
                        riptide.GoHang(),
                        new RoadRunnerDrive(21, 54, riptide.drive),
                        new RoadRunnerDrive(11, 0, riptide.drive),
                        new InstantCommand(() -> riptide.vertical.toggleClawState()),
                        new InstantCommand(() -> currentState = Task.RETRIEVE_SPECIMEN)
                ));
                currentState = Task.WAIT_FOR_TASK;
                break;
            case RETRIEVE_SPECIMEN:
                //untested but should be basically same thing as hang specimen in reverse
                opMode.schedule(new SequentialCommandGroup(
                        new RoadRunnerDrive(-6, 0, riptide.drive),
                        riptide.GoWall(),
                        new RoadRunnerDrive(-20, -52, riptide.drive),
                        new RoadRunnerDrive(-6, 0, riptide.drive),
                        new RoadRunnerDrive(-20, -52, riptide.drive),
                        new InstantCommand(() -> currentState = Task.HANG_SPECIMEN)
                ));
                currentState = Task.WAIT_FOR_TASK;
                break;

                // basket

            case PARK:
                currentState = Task.WAIT_FOR_TASK;
                break;
            case PRELOAD_BASKET_DRIVE:
                currentState = Task.WAIT_FOR_TASK;
                break;
            case RETRIEVE_SAMPLE:
                currentState = Task.WAIT_FOR_TASK;
                break;
            case DEPOSIT_SAMPLE:
                currentState = Task.WAIT_FOR_TASK;
                break;
            case PARK_BASKET:
                currentState = Task.WAIT_FOR_TASK;
                break;


            case WAIT_FOR_TASK:
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + currentState);
        }
    }
}
