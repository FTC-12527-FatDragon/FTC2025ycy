package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.HashMap;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommandBase;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;

@TeleOp(name = "DeltaHALTeleop")
public class DeltaCar extends CommandOpMode {
    private GamepadEx gamepadEx1;
    private Lift lift;
    private AlphaLiftClaw liftClaw;
    private AlphaSlide slide;
    private SampleMecanumDrive drive;

    private boolean isPureHandoffComplete = false;
    private boolean isHangComplete = false;
    private boolean aimFinished = false;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        lift = new Lift(hardwareMap, telemetry);
        liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
        slide = new AlphaSlide(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        slide.initialize();
        liftClaw.initialize();

        Command preHang = AutoCommandBase.toPreHang(lift, liftClaw);
        Command grab = AutoCommandBase.chamberToGrab(lift, liftClaw).alongWith(slide.aimCommand());

        Supplier<Command> stow =
                () ->
                        new ConditionalCommand(
                                new InstantCommand(),
                                liftClaw.openClawCommand(0)
                                        .andThen(liftClaw.foldLiftArmCommand(0))
                                        .alongWith(new InstantCommand(() -> liftClaw.stowWrist()))
                                        .andThen(lift.setGoalCommand(Lift.Goal.STOW, false))
                                        .andThen(new InstantCommand(() -> isHangComplete = false)),
                                () -> !isHangComplete);

        SelectCommand specimenCommands =
                new SelectCommand(
                        new HashMap<Object, Command>() {
                            {
                                put(Lift.Goal.GRAB, preHang);
                                put(Lift.Goal.STOW, grab);
                                put(Lift.Goal.HANG, stow.get());
                            }
                        },
                        () -> lift.getGoal());

        Supplier<Command> handoffCommand =
                () ->
                        slide
                                .handoffCommand()
                                .alongWith(liftClaw.openClawCommand())
                                .andThen(liftClaw.closeClawCommand())
                                .andThen(new InstantCommand(() -> slide.openIntakeClaw()));

        // Drive
        drive.setDefaultCommand(
                new TeleopDriveCommand(
                        drive,
                        () -> gamepadEx1.getLeftY(),
                        () -> -gamepadEx1.getLeftX(),
                        () -> -gamepadEx1.getRightX(),
                        () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON),
                        () -> gamepadEx1.getButton(GamepadKeys.Button.START),
                        () -> false));

        // Basket Up
        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new ConditionalCommand(
                                new InstantCommand(),
                                new ParallelCommandGroup(
                                        slide.aimCommand(),
                                        lift.setGoalCommand(Lift.Goal.BASKET, false),
                                        new WaitUntilCommand(() -> lift.getCurrentPosition() > 600)
                                                .andThen(new InstantCommand(liftClaw::upLiftArm)
                                                        .alongWith(new InstantCommand(liftClaw::basketWrist)))),
                                () -> !isPureHandoffComplete));


        // Basket Down
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(
                        new ConditionalCommand(
                                liftClaw.openClawCommand(),
                                new SequentialCommandGroup(
                                        liftClaw.foldLiftArmCommand(),
                                        new InstantCommand(liftClaw::stowWrist),
                                        new WaitCommand(100),
                                        lift.setGoalCommand(Lift.Goal.STOW, false),
                                        new InstantCommand(() -> isPureHandoffComplete = false)),
                                liftClaw::getLiftClawPos
                        ).andThen(new InstantCommand(() -> aimFinished = true)));

        // Chamber Grab
        new FunctionalButton(
                () ->
                        gamepadEx1.getButton(GamepadKeys.Button.B)
                                && lift.getGoal() == Lift.Goal.GRAB)
                .whenPressed(liftClaw.switchLiftClawCommand());

        // ChamberUp / Down
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                        lift.setGoalCommand(Lift.Goal.HANG)
                                .andThen(new InstantCommand(() -> isHangComplete = true)),
                        lift.setGoalCommand(Lift.Goal.PRE_HANG)
                                .andThen(new InstantCommand(() -> isHangComplete = false)),
                        () -> lift.getGoal() == Lift.Goal.PRE_HANG
                )
        );

        // STOW Connect
        new FunctionalButton(
                () ->
                        gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
                                && slide.getGoal() == AlphaSlide.Goal.AIM
                                && lift.getGoal() == Lift.Goal.STOW)
                .whenPressed(
                        new ConditionalCommand(
                                new InstantCommand(() -> slide.preHandoffSlideExtension()),
                                new InstantCommand(),
                                () -> !slide.isSlideForward())
                                .alongWith(
                                        new InstantCommand(() -> slide.handoffWristTurn())
                                                .alongWith(
                                                        handoffCommand.get()
                                                                .andThen(new WaitCommand(50))
                                                                .andThen(new InstantCommand(() -> isPureHandoffComplete = true)))),
                        false);

        // Reset to Stow
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                slide.aimCommand()
                        .alongWith(new InstantCommand(() -> aimFinished = true))
                        .alongWith(new SequentialCommandGroup(
                                liftClaw.foldLiftArmCommand(),
                                new InstantCommand(liftClaw::stowWrist),
                                new WaitCommand(100),
                                lift.setGoalCommand(Lift.Goal.STOW, false)))
                );

        // Chamber Next Mode
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        specimenCommands
                        .andThen(new InstantCommand(() -> aimFinished = true))
                        , false);


        // Reset Slides
        new FunctionalButton(
                () ->
                        gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                                && lift.getGoal() == Lift.Goal.STOW
                                && (currentRobot == DriveConstants.RobotType.ALPHA ? true : slide.getSlideServo() == AlphaSlide.SlideServo.BACK))
                .whenHeld(
                        currentRobot == DriveConstants.RobotType.ALPHA ?
                                lift.manualResetCommand() :
                                lift.manualResetCommand().alongWith(slide.manualResetCommand())
                );
        // Grab
        new FunctionalButton(
                () ->
                        gamepadEx1.getButton(GamepadKeys.Button.A)
                                && slide.getGoal() == AlphaSlide.Goal.AIM && aimFinished)
                .whenPressed(slide.grabCommand()
                        .andThen(new InstantCommand(() -> aimFinished = false)));

        // Aim
        new FunctionalButton(
                () ->
                        gamepadEx1.getButton(GamepadKeys.Button.A)
                                && slide.getGoal() != AlphaSlide.Goal.AIM && !aimFinished)
                .whenPressed(
                        slide.aimCommand().andThen(new InstantCommand(() -> aimFinished = true))
                );

        new FunctionalButton(
                () ->
                        gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                                && slide.getGoal() != AlphaSlide.Goal.HANDOFF)
                .whenPressed(new InstantCommand(slide::forwardSlideExtension));


        new FunctionalButton(
                () ->
                        gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                                && slide.getGoal() != AlphaSlide.Goal.HANDOFF)
                .whenPressed(new InstantCommand(slide::backwardSlideExtension));


        new FunctionalButton(
                () ->
                        gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                                && slide.getGoal() == AlphaSlide.Goal.AIM)
                .whenPressed(new InstantCommand(() -> slide.leftTurnServo()));


        new FunctionalButton(
                () ->
                        gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                                && slide.getGoal() == AlphaSlide.Goal.AIM)
                .whenPressed(new InstantCommand(() -> slide.rightTurnServo()));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        lift.periodicTest();
        telemetry.update();
    }
}
