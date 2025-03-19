package org.firstinspires.ftc.teamcode.riptide.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.RiptideConstants;
import org.firstinspires.ftc.teamcode.riptide.commands.HorizontalSlideCommand;
import org.firstinspires.ftc.teamcode.riptide.commands.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.riptide.commands.RoadRunnerTurn;
import org.firstinspires.ftc.teamcode.riptide.subsystems.HorizontalSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.VerticalSubsystem;

public class RiptideAuto {

    public Riptide riptide;

    private CommandOpMode opMode;
    Task currentState = Task.PRELOAD_SPECIMEN;

    public Pose2d desiredPosition;

    private enum Task{
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
        riptide = new Riptide(opMode, Riptide.OpModeType.AUTO, allianceColor);
        riptide.setStartPosition(startingPosition, allianceColor);
        riptide.target = target;
        if(riptide.target == Riptide.Target.SPECIMENS)
        {
            currentState = Task.PRELOAD_SPECIMEN;
        } else
        {
            currentState = Task.PRELOAD_BASKET;
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
                                new WaitCommand(750),
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
                        riptide.GoWall(),
                        new SequentialCommandGroup(
                                new RoadRunnerDrive(0, -27, riptide.drive), // y -28
                                new RoadRunnerDrive(30, -10, riptide.drive),
                                new RoadRunnerDrive(-46, -4, riptide.drive),
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
                                        new RoadRunnerDrive(2.5, -21, 100, riptide.drive),
                                    new HorizontalSlideCommand(riptide.horizontal, RiptideConstants.HORIZONTAL_SLIDE_MAX),
                                    riptide.horizontal.changeServos(HorizontalSubsystem.Position.SUB)
                                ),
                                new InstantCommand(() -> {
                                    riptide.horizontal.SetClawDownState(HorizontalSubsystem.DownState.DOWN);
                                    riptide.horizontal.wrist.setPosition(.25);
                                }),
                                new InstantCommand(() -> riptide.horizontal.Grab()),
                                new RoadRunnerTurn(-80, riptide.drive),
                                new InstantCommand(() -> {
                                    riptide.horizontal.SetClaw(HorizontalSubsystem.GripState.OPEN);
                                    riptide.horizontal.SetClawDownState(HorizontalSubsystem.DownState.UP);
                                }),
                                //first sample deposited
                                new RoadRunnerTurn(90, riptide.drive),
                                new RoadRunnerDrive(0, -6, riptide.drive),
                                new InstantCommand(() -> riptide.horizontal.Grab()),
                                new RoadRunnerTurn(-90, riptide.drive),
                                new InstantCommand(() -> {
                                    riptide.horizontal.SetClaw(HorizontalSubsystem.GripState.OPEN);
                                    riptide.horizontal.SetClawDownState(HorizontalSubsystem.DownState.UP);
                                }),
                                //second sample deposited
                                new RoadRunnerTurn(90, riptide.drive),
                                new RoadRunnerDrive(0, -8, riptide.drive),
                                new InstantCommand(() -> riptide.horizontal.Grab()),
                                new RoadRunnerTurn(-90, riptide.drive),
                                new InstantCommand(() -> {
                                    riptide.horizontal.SetClaw(HorizontalSubsystem.GripState.OPEN);
                                    riptide.horizontal.SetClawDownState(HorizontalSubsystem.DownState.UP);
                                })
                                //third sample deposited
                                //add stuff to positions for first specimen
                        ));
                currentState = Task.WAIT_FOR_TASK;
                break;
            case HANG_SPECIMEN:
                opMode.schedule(
                        riptide.GoHang(),
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                        new RoadRunnerDrive(4, 0, riptide.drive),
                        new RoadRunnerDrive(18, 40 + (additionalCycles * 3), riptide.drive),
                        new RoadRunnerDrive(18, 0, riptide.drive),
                        new InstantCommand(() -> {
                            riptide.vertical.toggleClawState();
                            riptide.vertical.changePositionTo(VerticalSubsystem.Position.WALL);
                        }),
                        new InstantCommand(() -> currentState = Task.RETRIEVE_SPECIMEN)
                ));
                currentState = Task.WAIT_FOR_TASK;
                break;
            case RETRIEVE_SPECIMEN:
                additionalCycles++;
                opMode.schedule(
                        riptide.GoWall(),
                        new SequentialCommandGroup(
                        new RoadRunnerDrive(-6, 0, riptide.drive),
                        new RoadRunnerDrive(-18, -42, riptide.drive),
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
