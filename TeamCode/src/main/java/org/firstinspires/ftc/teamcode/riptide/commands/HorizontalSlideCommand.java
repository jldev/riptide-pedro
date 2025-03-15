package org.firstinspires.ftc.teamcode.riptide.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.riptide.subsystems.HorizontalSubsystem;

public class HorizontalSlideCommand extends CommandBase {

    private final HorizontalSubsystem system;
    private HorizontalSubsystem.Position position = null;

    private int specifiedPos;

    public HorizontalSlideCommand(HorizontalSubsystem system, HorizontalSubsystem.Position position) {
        this.system = system;
        this.position = position;

        addRequirements(system);
    }
    public HorizontalSlideCommand(HorizontalSubsystem system, int position) {
        this.system = system;
        this.specifiedPos = position;

        addRequirements(system);
    }

    @Override
    public void initialize() {
       if(position == null){
           position = HorizontalSubsystem.Position.SPECIFIED;
           this.system.changeToSlidePosition(specifiedPos);
       }else {
           this.system.changeToSlidePosition(position);
       }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || system.AtSetPosition(position);
    }
}
