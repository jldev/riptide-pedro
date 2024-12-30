package org.firstinspires.ftc.teamcode.riptide.opmodes;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.riptide.Riptide;
import org.firstinspires.ftc.teamcode.riptide.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.riptide.subsystems.KrakenEyeSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.SlideSubsystem;


//values


@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Riptide riptide;

    @Override
    public void initialize() {
        riptide = new Riptide(this, Riptide.OpModeType.TELEOP, Riptide.AllianceColor.RED);

//        this.schedule(new RunCommand(() -> {
//            helix.slides.addTelemetry(telemetry);
//            helix.pivot.addTelemetry(telemetry);
//            telemetry.update();
//        }));

        // Drive control
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(

//                neptune.drive, () -> -neptune.driverOp.getLeftY(),
//                neptune.driverOp::getLeftX, neptune.driverOp::getRightX,

                riptide.drive, () -> riptide.driverOp.getRightY(),
                () -> riptide.driverOp.getRightX(), () -> riptide.driverOp.getLeftX()
        );
        riptide.drive.setDefaultCommand(driveCommand);






        //         SLIDES

        // Manual Slides Button
        riptide.verticleSlideUp.whileHeld(new InstantCommand(() -> {
            riptide.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.UP);}));
        riptide.verticleSlideUp.whenReleased(new InstantCommand(() -> {
            riptide.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.OFF);}));
        riptide.verticleSlideDown.whileHeld(new InstantCommand(() -> {
            riptide.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.DOWN);}));
        riptide.verticleSlideDown.whenReleased(new InstantCommand(() -> {
            riptide.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.OFF);}));

        riptide.horizontalSlideOut.whileHeld(new InstantCommand(() -> {
            riptide.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.OUT);}));
        riptide.horizontalSlideOut.whenReleased(new InstantCommand(() -> {
            riptide.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.OFF);}));
        riptide.horizontalSlideIn.whileHeld(new InstantCommand(() -> {
            riptide.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.IN);}));
        riptide.horizontalSlideIn.whenReleased(new InstantCommand(() -> {
            riptide.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.OFF);}));



        //  Presets
        riptide.home_slidePreset.whenPressed(riptide.GoSub());

        riptide.wall_slidePreset.whenPressed(riptide.GoWall());

        riptide.hang_slidePreset.whenPressed(riptide.GoHang());

        riptide.basket_slidePreset.whenPressed(riptide.GoBasket());


        //    HANG
//        helix.hangRaise.whileHeld(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.UP);}));
//        helix.hangRaise.whenReleased(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.OFF);}));
//        helix.hangLower.whileHeld(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.DOWN);}));
//        helix.hangLower.whenReleased(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.OFF);}));


        //   PIVOT
        riptide.pivotRaise.whileHeld(new InstantCommand(() -> {
            riptide.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.UP);}));
        riptide.pivotRaise.whenReleased(new InstantCommand(() -> {
            riptide.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.OFF);}));
        riptide.pivotLower.whileHeld(new InstantCommand(() -> {
            riptide.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.DOWN);}));
        riptide.pivotLower.whenReleased(new InstantCommand(() -> {
            riptide.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.OFF);}));

        // Pivot Presets
        riptide.home_pivotPreset.whenPressed(new InstantCommand(() -> {
            riptide.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.HOME);}));
        riptide.hang_pivotPreset.whenPressed(new InstantCommand(() -> {
            riptide.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.HANG);}));
        riptide.basket_pivotPreset.whenPressed(new InstantCommand(() -> {
            riptide.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.BASKET);}));
        riptide.sub_pivotPreset.whenPressed(new InstantCommand(() -> {
            riptide.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.SUB);}));


        // Claw
        riptide.cycleDesiredSampleColor.whenPressed
                ( new InstantCommand(() -> {
                        switch (riptide.krakenEye.desiredColor) {
                            case YELLOW:
                                riptide.krakenEye.desiredColor = KrakenEyeSubsystem.SampleColor.RED;
                                break;
                            case RED:
                                riptide.krakenEye.desiredColor = KrakenEyeSubsystem.SampleColor.BLUE;
                                break;
                            case BLUE:
                                riptide.krakenEye.desiredColor = KrakenEyeSubsystem.SampleColor.YELLOW;
                                break;
                        }}
                ));
    }

}
