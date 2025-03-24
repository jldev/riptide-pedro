package org.firstinspires.ftc.teamcode.riptide;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.riptide.subsystems.HorizontalSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.VerticalSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Riptide {

    public final OpModeType mOpModeType;

    public final MecanumDriveSubsystem drive;

    public GamepadEx driverOp;
    public GamepadEx gunnerOp;
    public Pose2d currentPos;


    public final SwitchReader magSwitchButton1;
    public final SwitchReader magSwitchButton3;

    //subsystems
    public final VerticalSubsystem vertical;
    public final HorizontalSubsystem horizontal;

    public enum FieldPos {
        AU,
        BD
    }

    public FieldPos fieldPos;
    public AllianceColor allianceColor;

    public final CommandOpMode mOpMode;
    public enum AllianceColor {
        RED,
        BLUE
    }
    public enum Target {
        SPECIMENS,
        SAMPLES
    }
    public Target target = Target.SPECIMENS;
    public boolean pushSamples = true;

    //           BUTTONSSSSS
    // Gunner
    public GamepadButton verticleSlideUp;
    public GamepadButton verticleSlideDown;
    public GamepadButton horizontalSlideOut;
    public GamepadButton horizontalSlideIn;

    public GamepadButton home_slidePreset;
    public GamepadButton wall_slidePreset;
    public GamepadButton hang_slidePreset;
    public GamepadButton basket_slidePreset;
    public GamepadTriggerAsButton horizontalClawButton;
    public GamepadTriggerAsButton verticalClawButton;

    // Driver



    public enum OpModeType {
        TELEOP,
        AUTO
    }

    public Riptide(CommandOpMode opMode, OpModeType opModeType, AllianceColor ac) {
        mOpMode = opMode;
        mOpModeType = opModeType;
        allianceColor = ac;
        Pose2d initialPose = new Pose2d(12, -62, Math.toRadians(90));
        drive = new MecanumDriveSubsystem(new MecanumDrive(opMode.hardwareMap, initialPose), false);
        driverOp = new GamepadEx(opMode.gamepad1);
        gunnerOp = new GamepadEx(opMode.gamepad2);

        //     vertical
        vertical = new VerticalSubsystem(this,
                new MotorEx(opMode.hardwareMap, "vertSlide1", Motor.GoBILDA.RPM_312),
                new MotorEx(opMode.hardwareMap, "vertSlide2", Motor.GoBILDA.RPM_312),
                opMode,
                RiptideConstants.SLIDES_PID_POS_COEFFICIENT,
                RiptideConstants.SLIDES_PID_TOLERANCE,
                opMode.hardwareMap.get(Servo.class, "shoulder1"),
                opMode.hardwareMap.get(Servo.class, "shoulder2"),
                opMode.hardwareMap.get(Servo.class, "rotation"),
                opMode.hardwareMap.get(Servo.class, "vElbow"),
                opMode.hardwareMap.get(Servo.class, "vGrip"),
                opMode.hardwareMap.get(Servo.class, "speedSwitch")
                );

        //     horizontal
        horizontal = new HorizontalSubsystem(this,
                new MotorEx(opMode.hardwareMap, "horzSlide", Motor.GoBILDA.RPM_435),
                opMode,
                RiptideConstants.SLIDES_PID_POS_COEFFICIENT,
                RiptideConstants.SLIDES_PID_TOLERANCE,
                opMode.hardwareMap.get(Servo.class, "shoulder"),
                opMode.hardwareMap.get(Servo.class, "hElbow"),
                opMode.hardwareMap.get(Servo.class, "wrist"),
                opMode.hardwareMap.get(Servo.class, "hGrip")
        );

        // pseudo buttons
        magSwitchButton1 = new SwitchReader(opMode.hardwareMap, false, "vSwitch1");
        magSwitchButton1.whenActive(new InstantCommand(vertical::stopMotorResetEncoder));

        magSwitchButton3 = new SwitchReader(opMode.hardwareMap, false, "hSwitch");
        magSwitchButton3.whenActive(new InstantCommand(horizontal::stopMotorResetEncoder));

//        if(mOpModeType == Riptide.OpModeType.AUTO) {
//            vertical.homeSlides(magSwitchButton1, magSwitchButton2);
//            horizontal.homeSlides(magSwitchButton3);
//        }

        opMode.register(vertical);
        opMode.register(horizontal);

        //       gunner setup
        //slide manual
        verticleSlideUp = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_UP);
        verticleSlideDown = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_DOWN);

        horizontalSlideOut = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_LEFT);
        horizontalSlideIn = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_RIGHT);

//        horizontalClawDown = new GamepadButton(gunnerOp, GamepadKeys.Button.LEFT_BUMPER);

           // presets
        home_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.A);
        wall_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.X);
        hang_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.B);
        basket_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.Y);

        horizontalClawButton = new GamepadTriggerAsButton(gunnerOp, GamepadKeys.Trigger.LEFT_TRIGGER, 0.5);
        verticalClawButton = new GamepadTriggerAsButton(gunnerOp, GamepadKeys.Trigger.RIGHT_TRIGGER, 0.5);
        //     driver setup

    }



    public void setStartPosition(Pose2d pos){
        this.currentPos = pos;
        drive.setPoseEstimate(this.currentPos);
    }
        //Start positions for each auto placement
    public void setStartPosition(FieldPos fp, AllianceColor ac) {

        this.fieldPos = fp;
        this.allianceColor = ac;

        this.currentPos = new Pose2d(12, -62, Math.toRadians(90));

        drive.setPoseEstimate(this.currentPos);
    }

    public Command GoSub() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.HOME)),
                new InstantCommand(() -> horizontal.changeToSlidePosition(HorizontalSubsystem.Position.SUB)),
                horizontal.changeServos(HorizontalSubsystem.Position.SUB),
                vertical.changeServos(VerticalSubsystem.Position.HOME)


        );
    }

    public Command GoHang() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    vertical.setClawImmediate(VerticalSubsystem.GripState.CLOSED);
                }),
                new WaitCommand(100),
                new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.HANG)),
                new InstantCommand(() -> horizontal.changeToSlidePosition(HorizontalSubsystem.Position.HOME)),
                new WaitCommand(150),
                vertical.changeServos(VerticalSubsystem.Position.HANG),
                horizontal.changeServos(HorizontalSubsystem.Position.HOME)
        );
    }

    public Command GoWall() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    vertical.setClawImmediate(VerticalSubsystem.GripState.OPEN);
                    horizontal.SetClaw(HorizontalSubsystem.GripState.OPEN);
                }),
                new WaitCommand(100),
                new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.WALL)),
                new InstantCommand(() -> horizontal.changeToSlidePosition(HorizontalSubsystem.Position.HOME)),
                horizontal.changeServos(HorizontalSubsystem.Position.HOME),
                new WaitCommand(200),
                vertical.changeServos(VerticalSubsystem.Position.WALL)
        );
    }

    public Command GoBasket() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    vertical.setClawImmediate(VerticalSubsystem.GripState.CLOSED);
                    horizontal.SetClaw(HorizontalSubsystem.GripState.OPEN);
                }),
                new WaitCommand(200),
                new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.BASKET)),
                new InstantCommand(() -> horizontal.changeToSlidePosition(HorizontalSubsystem.Position.HOME)),
                vertical.changeServos(VerticalSubsystem.Position.BASKET),
                new WaitCommand(500),
                horizontal.changeServos(HorizontalSubsystem.Position.HOME)

        );
    }

    public Command GoHandshake(){
        // we need to specify if were going to basket or dropping behind and maybe have a drop behind command here
        return new SequentialCommandGroup(
                vertical.changeServos(VerticalSubsystem.Position.HANDSHAKE),
                horizontal.changeServos(HorizontalSubsystem.Position.HANDSHAKE),
                new WaitCommand(1000),
                new InstantCommand(() -> {
                    vertical.changePositionTo(VerticalSubsystem.Position.HOME);
                    horizontal.changeToSlidePosition(HorizontalSubsystem.Position.HANDSHAKE);
                })
        );
    }


    public Command GoPreloadBasket() {
        return new SequentialCommandGroup(
                new WaitCommand(500),
                new InstantCommand(() -> {
                    vertical.changePositionTo(VerticalSubsystem.Position.PRELOAD_BASKET);})
        );
    }
}
