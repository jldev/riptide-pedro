package riptide;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import riptide.commands.HorizontalSlideCommand;
import riptide.commands.ServoPositionCommand;
import riptide.opmodes.RiptideAuto;
import riptide.subsystems.HorizontalSubsystem;
import riptide.subsystems.VerticalSubsystem;

public class Riptide {

    public final OpModeType mOpModeType;

//    public final MecanumDriveSubsystem drive;
    public final Follower follower;

    public GamepadEx driverOp;
    public GamepadEx gunnerOp;
    public Pose currentPos;


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
    public GamepadButton speed_switch_switcher;
    public GamepadTriggerAsButton horizontalClawButton;
    public GamepadTriggerAsButton verticalClawButton;

    // Driver



    public enum OpModeType {
        TELEOP,
        AUTO
    }
public RiptideAuto auto;

    public Riptide(CommandOpMode opMode, OpModeType opModeType, AllianceColor ac, RiptideAuto auto) {
        this(opMode, opModeType, ac);
    this.auto = auto;
    }

    public Riptide(CommandOpMode opMode, OpModeType opModeType, AllianceColor ac) {
        mOpMode = opMode;
        mOpModeType = opModeType;
        allianceColor = ac;
//        Pose2d initialPose = new Pose2d(12, -62, Math.toRadians(90));
//        drive = new MecanumDriveSubsystem(new MecanumDrive(opMode.hardwareMap, initialPose), false);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(opMode.hardwareMap);
        driverOp = new GamepadEx(opMode.gamepad1);
        gunnerOp = new GamepadEx(opMode.gamepad2);

        //     vertical
        vertical = new VerticalSubsystem(this,
                new MotorEx(opMode.hardwareMap, "vertSlide1", Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, "vertSlide2", Motor.GoBILDA.RPM_435),
                opMode,
                RiptideConstants.SLIDES_PID_POS_COEFFICIENT,
                RiptideConstants.SLIDES_PID_TOLERANCE,
                opMode.hardwareMap.get(Servo.class, "shoulder1"),
                opMode.hardwareMap.get(Servo.class, "shoulder2"),
                opMode.hardwareMap.get(Servo.class, "vWrist"),
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
                opMode.hardwareMap.get(Servo.class, "hGrip"),
                opMode.hardwareMap.get(Servo.class, "hockey")
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

        speed_switch_switcher = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP);

    }



    public void setStartPosition(Pose pos){
        this.currentPos = pos;
        follower.setStartingPose(this.currentPos);
    }
        //Start positions for each auto placement
    public void setStartPosition(FieldPos fp, AllianceColor ac) {

        this.fieldPos = fp;
        this.allianceColor = ac;

        this.currentPos = new Pose(12, -62, Math.toRadians(90));
        follower.setStartingPose(this.currentPos);
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
        if(mOpModeType == OpModeType.AUTO){
            if(auto.currentState == RiptideAuto.Task.PRELOAD_SPECIMEN){
                return new ParallelCommandGroup(
                        new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.HANG)),
                        vertical.changeServos(VerticalSubsystem.Position.HANG),
                        new InstantCommand(() -> horizontal.changeToSlidePosition(HorizontalSubsystem.Position.HOME)),
                        horizontal.changeServos(HorizontalSubsystem.Position.HOME)
                );
            }
        }
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ServoPositionCommand(vertical.shoulder1, RiptideConstants.VERT_PICKUP_SHOULDER, true),
                        new ServoPositionCommand(vertical.shoulder2, RiptideConstants.VERT_PICKUP_SHOULDER, true),
                        new ServoPositionCommand(vertical.elbow, RiptideConstants.VERT_PICKUP_ELBOW, true)
                ),
                new InstantCommand(() -> {vertical.setClawImmediate(VerticalSubsystem.GripState.CLOSED);}),
                new WaitCommand(75),
                new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.HANG)),
                vertical.changeServos(VerticalSubsystem.Position.HANG),
                new InstantCommand(() -> horizontal.changeToSlidePosition(HorizontalSubsystem.Position.HOME)),
                horizontal.changeServos(HorizontalSubsystem.Position.HOME)
        );
    }

    public Command GoWall() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    vertical.setClawImmediate(VerticalSubsystem.GripState.OPEN);
                }),
                new InstantCommand(() -> horizontal.changeToSlidePosition(HorizontalSubsystem.Position.HOME)),
                new WaitCommand(50),
                horizontal.changeServos(HorizontalSubsystem.Position.HOME),
                new ParallelCommandGroup(
                        vertical.changeServos(VerticalSubsystem.Position.WALL),
                        new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.WALL))
                )
        );
    }

    public Command GoBasket() {
        if(mOpModeType == OpModeType.AUTO){
            if(auto.currentState == RiptideAuto.Task.PRELOAD_BASKET){
                return new SequentialCommandGroup(
                        new InstantCommand(() -> vertical.setClawImmediate(VerticalSubsystem.GripState.CLOSED)),
                        new WaitCommand(150),
                        new InstantCommand(() -> horizontal.SetClaw(HorizontalSubsystem.GripState.OPEN)),
                        new WaitCommand(150),
                        horizontal.changeServos(HorizontalSubsystem.Position.HOME),
                        new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.BASKET_AUTO)),
                        vertical.changeServos(VerticalSubsystem.Position.BASKET_AUTO),
                        new WaitCommand(200),
                        new HorizontalSlideCommand(horizontal, HorizontalSubsystem.Position.HOME));
            }
        }
        return new SequentialCommandGroup(
                new InstantCommand(() -> vertical.setClawImmediate(VerticalSubsystem.GripState.CLOSED)),
                new WaitCommand(150),
                new InstantCommand(() -> horizontal.SetClaw(HorizontalSubsystem.GripState.OPEN)),
                new WaitCommand(150),
                horizontal.changeServos(HorizontalSubsystem.Position.HOME),
                new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.BASKET)),
                vertical.changeServos(VerticalSubsystem.Position.BASKET),
                new WaitCommand(200),
                new HorizontalSlideCommand(horizontal, HorizontalSubsystem.Position.HOME)
        );
    }

    public Command GoHandshake(){
        // we need to specify if were going to basket or dropping behind and maybe have a drop behind command here
        return new SequentialCommandGroup(
                horizontal.changeServos(HorizontalSubsystem.Position.HANDSHAKE),
                vertical.changeServos(VerticalSubsystem.Position.HANDSHAKE),
                new InstantCommand(() -> vertical.changePositionTo(VerticalSubsystem.Position.HOME)),
                new WaitCommand(RiptideConstants.HANDSHAKE_WAIT_TIME),
                new HorizontalSlideCommand(horizontal, HorizontalSubsystem.Position.HANDSHAKE),
                new WaitCommand(RiptideConstants.HANDSHAKE_WAIT_TIME),
                GoBasket()
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
