package org.firstinspires.ftc.teamcode.riptide.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.RiptideConstants;
import org.firstinspires.ftc.teamcode.riptide.commands.HorizontalSlideCommand;
import org.firstinspires.ftc.teamcode.riptide.commands.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.riptide.commands.RoadRunnerTurn;
import org.firstinspires.ftc.teamcode.riptide.commands.ServoPositionCommand;

public class RiptideAuto {

    public Riptide riptide;

    private CommandOpMode opMode;
    public Task currentState = Task.PRELOAD_SPECIMEN;

    public Pose2d desiredPosition;

    public enum Task{
        // specimen
        PRELOAD_SPECIMEN,
        PUSH_SAMPLES,
        GRAB_SAMPLES,
        HANG_SPECIMEN,
        RETRIEVE_SPECIMEN,
        PARK,

        // basket
        PRELOAD_BASKET,
        RETRIEVE_SAMPLE,
        DEPOSIT_SAMPLE,
        PARK_BASKET,

        // also this
        WAIT_FOR_TASK
    }

    private  int additionalCycles;

    private int runCount = 0;

    public RiptideAuto(CommandOpMode commandOpMode, Riptide.FieldPos startingPosition, Riptide.AllianceColor allianceColor, Riptide.Target target) {
        opMode = commandOpMode;
        riptide = new Riptide(opMode, Riptide.OpModeType.AUTO, allianceColor, this);
        riptide.setStartPosition(startingPosition, allianceColor);
        riptide.target = target;
        if(riptide.target == Riptide.Target.SPECIMENS)
        {
            currentState = Task.PRELOAD_SPECIMEN;
        } else
        {
            currentState = Task.GRAB_SAMPLES;
        }
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(180));
        riptide.setStartPosition(startPos);
        additionalCycles = 0;
    }

    public void run() {
        opMode.telemetry.addData("Current State", currentState);
        opMode.telemetry.addData("Run Count", runCount++);
        opMode.telemetry.addLine(String.format("Pose X: %.2f, Y: %.2f, Rot: %.2f", riptide.drive.getPoseEstimate().position.x,
                riptide.drive.getPoseEstimate().position.y, Math.toDegrees(riptide.drive.getPoseEstimate().heading.toDouble())));
        opMode.telemetry.update();

        switch (currentState) {

            // specimen

            case PRELOAD_SPECIMEN:
                opMode.schedule(
                        riptide.GoHang(),
                        new SequentialCommandGroup(
                                new RoadRunnerDrive(32, 0, riptide.drive),
                                new InstantCommand(() -> riptide.vertical.toggleClawState()),
                                new RoadRunnerDrive(-9, 0, riptide.drive),

                                new InstantCommand(() -> currentState = Task.GRAB_SAMPLES)
                            )
                        );
                currentState = Task.WAIT_FOR_TASK;
                break;
            case PUSH_SAMPLES:
                opMode.schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        riptide.GoWall(),
                                        new RoadRunnerDrive(0, -27, riptide.drive)),
                                new RoadRunnerDrive(32, -12, riptide.drive),
                                new RoadRunnerDrive(-48, -4, riptide.drive),
                                new RoadRunnerDrive(43, 4, riptide.drive),
                                new RoadRunnerDrive(0, -12, riptide.drive),
                                new RoadRunnerDrive(-45, 10, riptide.drive),
                                new RoadRunnerDrive(-8, 0, riptide.drive),
                                new InstantCommand(() -> currentState = Task.HANG_SPECIMEN))
                );
                currentState = Task.WAIT_FOR_TASK;
                break;
            case GRAB_SAMPLES:
                opMode.schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RoadRunnerDrive(2.5, -22, 325, riptide.drive),
                                    new HorizontalSlideCommand(riptide.horizontal, RiptideConstants.HORIZONTAL_SLIDE_MAX)
                                ),
                                new ServoPositionCommand(riptide.horizontal.hockey, RiptideConstants.HOCKEY_DOWN, true),
                                new RoadRunnerTurn(-85, riptide.drive),
                                new InstantCommand(() -> riptide.horizontal.hockey.setPosition(RiptideConstants.HOCKEY_UP)),
                                //first sample deposited
                                new RoadRunnerDrive(0, -8, 320, riptide.drive),
                                new ServoPositionCommand(riptide.horizontal.hockey, RiptideConstants.HOCKEY_DOWN, true),
                                new RoadRunnerTurn(-80, riptide.drive),
                                new InstantCommand(() -> riptide.horizontal.hockey.setPosition(RiptideConstants.HOCKEY_UP)),
                                //second sample deposited
                                new RoadRunnerDrive(0, -7, 320, riptide.drive),
                                new ServoPositionCommand(riptide.horizontal.hockey, RiptideConstants.HOCKEY_DOWN, true),
                                new RoadRunnerTurn(-80, riptide.drive),
                                new InstantCommand(() -> riptide.horizontal.hockey.setPosition(RiptideConstants.HOCKEY_UP)),
                                //third sample deposited
                                new ParallelCommandGroup(
                                        new RoadRunnerDrive(0, 0, 180, riptide.drive),
                                        new HorizontalSlideCommand(riptide.horizontal, 0)
                                )
                                //add stuff to positions for first specimen
                        ));
                currentState = Task.WAIT_FOR_TASK;
                break;
            case HANG_SPECIMEN:
                opMode.schedule(
                        riptide.GoHang(),
                        new SequentialCommandGroup(
                        new RoadRunnerDrive(4, 0, riptide.drive),
                        new RoadRunnerDrive(18, 40 + (additionalCycles * 3), riptide.drive),
                        new RoadRunnerDrive(18, 0, riptide.drive),
                        new InstantCommand(() -> {
                            riptide.vertical.toggleClawState();
                        }),
                        new InstantCommand(() -> currentState = Task.RETRIEVE_SPECIMEN)
                ));
                currentState = Task.WAIT_FOR_TASK;
                break;
            case RETRIEVE_SPECIMEN:
                additionalCycles++;
                opMode.schedule(
                        new SequentialCommandGroup(
                        new RoadRunnerDrive(-6, 0, riptide.drive),
                        new ParallelCommandGroup(
                                riptide.GoWall(),
                                new RoadRunnerDrive(-14, -42, riptide.drive)
                        ),
                        new RoadRunnerDrive(-8, 0, riptide.drive),
                        new InstantCommand(() -> {
                            riptide.vertical.toggleClawState();
                            currentState = Task.HANG_SPECIMEN;
                        })
                ));
                currentState = Task.WAIT_FOR_TASK;
                break;


                // basket


            case PARK:
                currentState = Task.WAIT_FOR_TASK;
                break;
            case PRELOAD_BASKET:
                opMode.schedule(new SequentialCommandGroup(
                        riptide.GoBasket(),
                        new RoadRunnerDrive(0, -6, riptide.drive),
                        new InstantCommand(() -> riptide.vertical.toggleClawState())

                ));
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
