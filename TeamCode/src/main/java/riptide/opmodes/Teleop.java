package riptide.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import riptide.Riptide;
import riptide.subsystems.HorizontalSubsystem;
import riptide.subsystems.VerticalSubsystem;


//values


@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Riptide riptide;

    @Override
    public void initialize() {
        riptide = new Riptide(this, Riptide.OpModeType.TELEOP, Riptide.AllianceColor.RED);

        this.schedule(new RunCommand(() -> {
            riptide.vertical.addTelemetry(telemetry);
            telemetry.addLine("                this is a space");
            riptide.horizontal.addTelemetry(telemetry);
            telemetry.addData("Mag switch 1 - ", riptide.magSwitchButton1.get());
            telemetry.addData("Mag switch 3 - ", riptide.magSwitchButton3.get());

            telemetry.update();
        }));

        // Drive control
//        MecanumDriveCommand driveCommand = new MecanumDriveCommand(
//                riptide.drive, () -> -riptide.driverOp.getRightY(),
//                () -> -riptide.driverOp.getRightX(), () -> (-riptide.driverOp.getLeftX() * .6)
//        );
//        riptide.drive.setDefaultCommand(driveCommand);

        riptide.follower.startTeleopDrive();
        //         VERTICAL

        riptide.verticleSlideUp.whileHeld(new InstantCommand(() -> {
            riptide.vertical.verticalManualSlideControl(VerticalSubsystem.SlideManualControlDirection.UP);}));
        riptide.verticleSlideUp.whenReleased(new InstantCommand(() -> {
            riptide.vertical.verticalManualSlideControl(VerticalSubsystem.SlideManualControlDirection.OFF);}));
        riptide.verticleSlideDown.whileHeld(new InstantCommand(() -> {
            riptide.vertical.verticalManualSlideControl(VerticalSubsystem.SlideManualControlDirection.DOWN);}));
        riptide.verticleSlideDown.whenReleased(new InstantCommand(() -> {
            riptide.vertical.verticalManualSlideControl(VerticalSubsystem.SlideManualControlDirection.OFF);}));

        riptide.verticalClawButton.whenPressed(new InstantCommand(() -> riptide.vertical.toggleClawState()));

        riptide.speed_switch_switcher.whenPressed(new InstantCommand(() -> riptide.vertical.toggleMotorSpeed()));

        //         HORIZONTAL

        riptide.horizontalSlideOut.whileHeld(new InstantCommand(() -> {
            riptide.horizontal.horizontalManualSlideControl(HorizontalSubsystem.SlideManualControlDirection.UP);}));
        riptide.horizontalSlideOut.whenReleased(new InstantCommand(() -> {
            riptide.horizontal.horizontalManualSlideControl(HorizontalSubsystem.SlideManualControlDirection.OFF);}));
        riptide.horizontalSlideIn.whileHeld(new InstantCommand(() -> {
            riptide.horizontal.horizontalManualSlideControl(HorizontalSubsystem.SlideManualControlDirection.DOWN);}));
        riptide.horizontalSlideIn.whenReleased(new InstantCommand(() -> {
            riptide.horizontal.horizontalManualSlideControl(HorizontalSubsystem.SlideManualControlDirection.OFF);}));

        riptide.horizontalClawButton.whenPressed(new InstantCommand(() -> riptide.horizontal.ToggleClawState()));

        //  Presets
        riptide.home_slidePreset.whenPressed(riptide.GoSub());

        riptide.wall_slidePreset.whenPressed(riptide.GoWall());

        riptide.hang_slidePreset.whenPressed(riptide.GoHang());

        riptide.basket_slidePreset.whenPressed(riptide.GoHandshake());
    }

    @Override
    public void run(){
        super.run();

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        riptide.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        riptide.follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", riptide.follower.getPose().getX());
        telemetry.addData("Y", riptide.follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(riptide.follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }
}
