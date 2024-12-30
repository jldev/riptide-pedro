package org.firstinspires.ftc.teamcode.riptide.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.riptide.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.riptide.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.subsystems.MecanumDriveSubsystem;

public class RiptideAuto {

    public Riptide riptide;

    private CommandOpMode opMode;
    Task currentState = Task.PRELOAD_DRIVE;

    public Pose2d desiredPosition;

    private enum Task{
        PRELOAD_DRIVE,
        RETRIEVE_SPECIMEN,
        DEPOSIT_SPECIMEN,
        RETRIEVE_SAMPLE,
        DEPOSIT_SAMPLE,
        WAIT_FOR_DRIVE,
        PUSH_SAMPLES,
        PARK,
        PARK_BASKET
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
            currentState = Task.DEPOSIT_SAMPLE;
        }
        Pose2d startPos = new Pose2d(12, -60, Math.toRadians(270));
        riptide.setStartPosition(startPos);

//        helix.limelight.pipelineSwitch(0);
//        helix.limelight.start();
//        opMode.schedule(new CenterOnSpecimenCommand(helix));
    }

    public void run() {
        opMode.telemetry.addData("Current State", currentState);
        opMode.telemetry.addData("Run Count", runCount++);
        opMode.telemetry.update();
        switch (currentState) {

            case PRELOAD_DRIVE:
                opMode.schedule(
                        new SequentialCommandGroup(
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 15),
                                riptide.GoHang(),
                                new WaitCommand(1000),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 15).whenFinished(()->{
                                    currentState = Task.DEPOSIT_SPECIMEN;
                                })
                            )
                        );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case WAIT_FOR_DRIVE:
                break;
            case RETRIEVE_SPECIMEN:
                break;
            case DEPOSIT_SPECIMEN:
                riptide.claw.mGripState = ClawSubsystem.GripState.OPEN;
                if(riptide.pushSamples)
                {
                    currentState = Task.PUSH_SAMPLES;
                } else
                {
                    currentState = Task.PARK;
                }
                break;
            case RETRIEVE_SAMPLE:
                break;
            case DEPOSIT_SAMPLE:
                opMode.schedule(
                        new SequentialCommandGroup(
                                riptide.GoPreloadBasket(),
                                new WaitCommand(1500),
                                new InstantCommand(() -> riptide.claw.ChangeClawPositionTo(ClawSubsystem.ClawState.SUB)),
                                new WaitCommand(500),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 4.5),
                                new WaitCommand(500)
                                .whenFinished(() -> {
                                    riptide.claw.mGripState = ClawSubsystem.GripState.OPEN;
                                //currentState = Task.PARK_BASKET;
                            }),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 6),
                                riptide.GoSub()
                        )
                );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case PUSH_SAMPLES:
                opMode.schedule(
                        new SequentialCommandGroup(
                                riptide.GoWall(),
                                new WaitCommand(2000),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 10),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 33),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 34),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 10),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 48),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 48),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 12),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 50)                        )
                );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case PARK:
                opMode.schedule(
                        new SequentialCommandGroup(
                                riptide.GoWall(),
                                new WaitCommand(2000),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 18),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 48),
                                new InstantCommand(() -> {
                                    riptide.claw.mGripState = ClawSubsystem.GripState.OPEN;}),
                                new WaitCommand(500).whenFinished(() -> currentState = Task.PARK_BASKET)
                        )
                );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case PARK_BASKET:
                opMode.schedule(
                        new SequentialCommandGroup(
                        new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 6),
                        riptide.GoSub(),
                        new WaitCommand(1000),
                                new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 6),
                        new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.RIGHT, 50),
                        new SimpleDriveCommand(riptide.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 8)
                        )
                );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + currentState);
        }
    }
}
