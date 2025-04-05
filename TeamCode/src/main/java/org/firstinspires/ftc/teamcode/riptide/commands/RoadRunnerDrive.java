package org.firstinspires.ftc.teamcode.riptide.commands;

import android.util.ArraySet;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.riptide.subsystems.MecanumDriveSubsystem;

import java.util.Arrays;
import java.util.Set;

public class RoadRunnerDrive extends CommandBase {
    private Action action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    private double x,y;
    private Double heading = null;
    private MecanumDriveSubsystem drive;
    public RoadRunnerDrive(double x, double y, MecanumDriveSubsystem drive){
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.requirements = new ArraySet<>(Arrays.asList(drive));
    }

    public RoadRunnerDrive(double x, double y, double heading, MecanumDriveSubsystem drive){
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.requirements = new ArraySet<>(Arrays.asList(drive));
    }

    @Override
    public void initialize(){
        if(heading == null)
        {
            action = drive.drive.actionBuilder(drive.drive.pose).strafeToConstantHeading(new Vector2d(drive.drive.pose.position.x + x, drive.drive.pose.position.y + y)).build();
        } else
        {
            action = drive.drive.actionBuilder(drive.drive.pose).strafeToLinearHeading(new Vector2d(drive.drive.pose.position.x + x, drive.drive.pose.position.y + y), Math.toRadians(heading)).build();
        }
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
