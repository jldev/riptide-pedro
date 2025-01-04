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

import org.firstinspires.ftc.teamcode.riptide.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.HorizontalSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.riptide.subsystems.VerticalSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Riptide {

    public final OpModeType mOpModeType;

    public final MecanumDriveSubsystem drive;
//    public final Limelight3A limelight;

    public GamepadEx driverOp;
    public GamepadEx gunnerOp;
    public Pose2d currentPos;


//    public final SwitchReader magSwitchButton;

    //subsystems
    public final VerticalSubsystem vertical;
    public final HorizontalSubsystem horizontal;
//    public final HangSubsystem hang;
    public final ClawSubsystem claw;
//    public final KrakenEyeSubsystem krakenEye;


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

    public GamepadButton instakeGripperButton;
    public GamepadButton intakeLiftButton;

    public GamepadButton verticleSlideUp;
    public GamepadButton verticleSlideDown;
    public GamepadButton horizontalSlideOut;
    public GamepadButton horizontalSlideIn;

    public GamepadButton home_slidePreset;
    public GamepadButton wall_slidePreset;
    public GamepadButton hang_slidePreset;
    public GamepadButton basket_slidePreset;

    public GamepadButton cycleDesiredSampleColor;


    // Driver

    public GamepadButton hangRaise;
    public GamepadButton hangLower;
    public GamepadButton pivotRaise;
    public GamepadButton pivotLower;

    public GamepadButton home_pivotPreset;
    public GamepadButton hang_pivotPreset;
    public GamepadButton basket_pivotPreset;
    public GamepadButton sub_pivotPreset;






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
//        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");



        //     vertical
        vertical = new VerticalSubsystem(this,
                new MotorEx(opMode.hardwareMap, "vertSlide1", Motor.GoBILDA.RPM_312),
                new MotorEx(opMode.hardwareMap, "vertSlide2", Motor.GoBILDA.RPM_312),
                opMode,
                RiptideConstants.SLIDES_PID_POS_COEFFICIENT,
                RiptideConstants.SLIDES_PID_TOLERANCE
                );

        //     horizontal
        horizontal = new HorizontalSubsystem(this,
                new MotorEx(opMode.hardwareMap, "horzSlide", Motor.GoBILDA.RPM_1620),
                opMode,
                RiptideConstants.SLIDES_PID_POS_COEFFICIENT,
                RiptideConstants.SLIDES_PID_TOLERANCE
        );




        //     pivot
//        pivot = new PivotSubsystem(this,
//                new MotorEx(opMode.hardwareMap, "pivotMotor", Motor.GoBILDA.RPM_223),
//                opMode,
//                RiptideConstants.SLIDES_PID_POS_COEFFICIENT,
//                RiptideConstants.SLIDES_PID_TOLERANCE
//        );




        //     hang
//        hang = new HangSubsystem(this,
//                new MotorEx(opMode.hardwareMap, "hangMotor", Motor.GoBILDA.RPM_435),
//                opMode,
//                HelixConstants.SLIDES_PID_POS_COEFFICIENT,
//                HelixConstants.SLIDES_PID_TOLERANCE
//        );



        //     claw
        claw = new ClawSubsystem(this,
                opMode,
                opMode.hardwareMap.get(Servo.class, "yaw_1"),
                opMode.hardwareMap.get(Servo.class, "pitch_2"),
                opMode.hardwareMap.get(Servo.class, "grip_3"));
//                opMode.hardwareMap.get(Limelight3A.class, "limelight"));


//        krakenEye = new KrakenEyeSubsystem(this, mOpMode, claw, limelight);



        // pseudo buttons
//        magSwitchButton = new SwitchReader(opMode.hardwareMap, false, "vSwitch");
//        magSwitchButton.whenPressed(new InstantCommand(slides::stopMotorResetEncoder));

        opMode.register(vertical);
        opMode.register(horizontal);
//        opMode.register(hang);
        opMode.register(claw);
//        opMode.register(krakenEye);





        //       gunner setup

           //intake
//        instakeGripperButton =  new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_RIGHT);
//        intakeLiftButton =  new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_LEFT);     // these are temp - gunner's out of buttons

           //slide manual
        verticleSlideUp = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_UP);
        verticleSlideDown = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_DOWN);

        horizontalSlideOut = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_LEFT);
        horizontalSlideIn = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_RIGHT);

           //slidePresets
        home_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.A);
        wall_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.X);
        hang_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.B);
        basket_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.Y);

          //claw
        // yaw = LTx
        // pitch = LTy
        // grip = RTy

        cycleDesiredSampleColor = new GamepadButton(gunnerOp, GamepadKeys.Button.LEFT_BUMPER);



        //     driver setup

           //hang
        hangRaise = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_LEFT);
        hangLower = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_RIGHT);
        // !!we should change this to have a button to go all the way up, and a button to go all the way down


           //pivot manual
//        pivotRaise = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_UP);
//        pivotLower = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_DOWN);

           //pivotPresets
        home_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.X);
        hang_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.B);
        basket_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.Y);
        sub_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.A);


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
                new InstantCommand(() -> {
                    vertical.changeToSlidePosition(VerticalSubsystem.Position.HOME);
                }),
                new WaitCommand(300),
                new InstantCommand(() -> claw.ChangeClawPositionTo(ClawSubsystem.ClawState.SUB))
        );
    }

    public Command GoHang() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    vertical.changeToSlidePosition(VerticalSubsystem.Position.HANG);

                }),
                new WaitCommand(500),
                new InstantCommand(() -> claw.ChangeClawPositionTo(ClawSubsystem.ClawState.HANG))
        );
    }

    public Command GoWall() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    vertical.changeToSlidePosition(VerticalSubsystem.Position.WALL);
                }),
                new WaitCommand(300),
                new InstantCommand(() -> claw.ChangeClawPositionTo(ClawSubsystem.ClawState.HOME))
        );
    }

    public Command GoBasket() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    vertical.changeToSlidePosition(VerticalSubsystem.Position.BASKET);

                }),
                new WaitCommand(500),
                new InstantCommand(() -> claw.ChangeClawPositionTo(ClawSubsystem.ClawState.BASKET))
        );
    }


    public Command GoPreloadBasket() {
        return new SequentialCommandGroup(
                new WaitCommand(500),
                new InstantCommand(() -> {
                    claw.ChangeClawPositionTo(ClawSubsystem.ClawState.BASKET);
                    vertical.changeToSlidePosition(VerticalSubsystem.Position.PRELOAD_BASKET);})
        );
    }
}
