package org.firstinspires.ftc.teamcode.riptide.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.riptide.subsystems.MecanumDriveSubsystem;

public class SimpleDriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final MecanumDriveSubsystem.DriveDirection direction;
    private final double distanceInches;


    public SimpleDriveCommand(MecanumDriveSubsystem drive, MecanumDriveSubsystem.DriveDirection direction, double inches ) {
        this.drive = drive;
        this.direction = direction;
        this.distanceInches = inches;

        addRequirements(drive);
    }

    @Override
    public void initialize() { this.drive.driveDirection(direction, distanceInches);}

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}
