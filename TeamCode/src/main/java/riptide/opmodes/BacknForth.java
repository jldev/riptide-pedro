package riptide.opmodes;

import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import riptide.Riptide;

@Config
@Autonomous(group = "drive", name = "BacknForth")
public class BacknForth extends CommandOpMode {
    Riptide riptide;
    private boolean started = false;
    @Override
    public void initialize() {
        riptide = new Riptide(this, Riptide.OpModeType.AUTO, Riptide.AllianceColor.BLUE);
//        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(180));
//        riptide.setStartPosition(startPos);
        started = false;
    }

    @Override
    public void run(){
        if(!started)
        {
            started = true;
//            this.schedule(
//                    new SequentialCommandGroup(
//                            new RoadRunnerDrive(48, 0, riptide.drive),
//                            new RoadRunnerDrive(-48, 0, riptide.drive),
//                            new RoadRunnerDrive(48, 0, riptide.drive),
//                            new RoadRunnerDrive(-48, 0, riptide.drive),
//                            new RoadRunnerDrive(48, 0, riptide.drive),
//                            new RoadRunnerDrive(-48, 0, riptide.drive)
//                    )
//            );
        }

//        telemetry.addLine(String.format("Pose X: %.2f, Y: %.2f, Rot: %.2f", riptide.drive.getPoseEstimate().position.x,
//                riptide.drive.getPoseEstimate().position.y, Math.toDegrees(riptide.drive.getPoseEstimate().heading.toDouble())));
        telemetry.update();
        super.run();
    }
}
