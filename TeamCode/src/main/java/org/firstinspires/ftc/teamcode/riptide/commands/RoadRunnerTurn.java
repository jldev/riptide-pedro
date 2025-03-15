package org.firstinspires.ftc.teamcode.riptide.commands;

import android.util.ArraySet;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.riptide.subsystems.MecanumDriveSubsystem;

import java.util.Arrays;
import java.util.Set;

public class RoadRunnerTurn extends CommandBase {
    private Action action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    private double degrees;
    private MecanumDriveSubsystem drive;
    public RoadRunnerTurn(double degrees, MecanumDriveSubsystem drive){
        this.drive = drive;
        this.degrees = degrees;
        this.requirements = new ArraySet<>(Arrays.asList(drive));
    }

    @Override
    public void initialize(){
        action = drive.drive.actionBuilder(drive.drive.pose).turn(Math.toRadians(degrees)).build();
    }
    @Override
    public Set<Subsystem> getRequirements(){
        return requirements;
    }

    @Override
    public void execute(){
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
