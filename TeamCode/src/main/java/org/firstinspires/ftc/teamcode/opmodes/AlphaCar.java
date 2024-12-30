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
import org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@TeleOp(name = "AlphaYCYTeleop")
public class AlphaCar extends CommandOpMode {
  private GamepadEx gamepadEx1;
  private Lift lift;
  private AlphaLiftClaw liftClaw;
  private AlphaSlide slide;
  private SampleMecanumDrive drive;

  private boolean isPureHandoffComplete = false;
  private boolean isHangComplete = false;

  @Override
  public void initialize() {
    gamepadEx1 = new GamepadEx(gamepad1);

    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);
    drive = new SampleMecanumDrive(hardwareMap);

    slide.initialize();
    liftClaw.initialize();
    lift.setGoal(Lift.Goal.STOW);

    // Teleop Drive Command
    drive.setDefaultCommand(
        new TeleopDriveCommand(
            drive,
            () -> gamepadEx1.getLeftY(),
            () -> -gamepadEx1.getLeftX(),
            () -> -gamepadEx1.getRightX(),
            () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON),
            () -> gamepadEx1.getButton(GamepadKeys.Button.START)));

    // Basket Up Command
    gamepadEx1
        .getGamepadButton(GamepadKeys.Button.X)
        .whenPressed(
            new ConditionalCommand(
                new InstantCommand(),
                new ParallelCommandGroup(
                    slide.aimCommand(),
                    new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
                    new WaitUntilCommand(() -> lift.getCurrentPosition() > 600)
                        .andThen(new InstantCommand(liftClaw::upLiftArm))),
                () -> !isPureHandoffComplete));

    // Basket Drop and Back
    gamepadEx1
        .getGamepadButton(GamepadKeys.Button.B)
        .whenPressed(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(() -> slide.slideArmDown())
                        .andThen(new WaitCommand(100))
                        .andThen(new InstantCommand(() -> slide.setGoal(AlphaSlide.Goal.AIM))),
                    new InstantCommand(),
                    () -> lift.getGoal() == Lift.Goal.HANG),
                new InstantCommand(liftClaw::openClaw),
                new InstantCommand(liftClaw::stowWrist),
                new WaitCommand(100),
                new InstantCommand(liftClaw::foldLiftArm),
                new WaitCommand(500),
                new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)),
                new InstantCommand(() -> isPureHandoffComplete = false)));

    // Aim
    gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(slide.aimCommand(), false);

    // Grab when aim
    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.A)
                    && slide.getGoal() == AlphaSlide.Goal.AIM)
        .whenPressed(slide.grabCommand(), false);

    // Pure Handoff
    Supplier<Command> handoffCommand =
        () ->
            slide
                .handoffCommand()
                .alongWith(new InstantCommand(liftClaw::openClaw))
                .andThen(new WaitCommand(600))
                .andThen(new InstantCommand(() -> liftClaw.closeClaw()))
                .andThen(new WaitCommand(250))
                .andThen(new InstantCommand(() -> slide.openIntakeClaw()));

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
                            handoffCommand
                                .get()
                                .andThen(new WaitCommand(50))
                                .andThen(new InstantCommand(() -> isPureHandoffComplete = true))),
                    new InstantCommand()),
            false);

    // Chamber Command from Grab
    Supplier<Command> preHang =
        () ->
            new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG))
                .alongWith(new InstantCommand(() -> liftClaw.chamberWrist()))
                .andThen(new InstantCommand(() -> liftClaw.chamberLiftArm()));

    Command grab = AutoCommand.chamberToGrab(lift, liftClaw).alongWith(slide.aimCommand());

    Supplier<Command> stow =
        () ->
            new ConditionalCommand(
                new InstantCommand(),
                new InstantCommand(() -> liftClaw.openClaw())
                    .andThen(new InstantCommand(() -> liftClaw.foldLiftArm()))
                    .alongWith(new InstantCommand(() -> liftClaw.stowWrist()))
                    .andThen(new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)))
                    .andThen(new InstantCommand(() -> isHangComplete = false)),
                () -> !isHangComplete);

    SelectCommand specimenCommands =
        new SelectCommand(
            new HashMap<Object, Command>() {
              {
                put(Lift.Goal.GRAB, preHang.get());
                put(Lift.Goal.STOW, grab);
                put(Lift.Goal.HANG, stow.get());
              }
            },
            () -> lift.getGoal());

    gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(specimenCommands, false);

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    && lift.getGoal() == Lift.Goal.PRE_HANG)
        .whenPressed(
            new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG))
                .andThen(new WaitCommand(300))
                .andThen(new InstantCommand(() -> isHangComplete = true)));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                    && lift.getGoal() == Lift.Goal.HANG)
        .whenPressed(
            new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG))
                .andThen(new WaitCommand(300))
                .andThen(new InstantCommand(() -> isHangComplete = false)));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                    && lift.getGoal() == Lift.Goal.GRAB)
        .whenPressed(new InstantCommand(() -> liftClaw.switchLiftClaw()));

    //        // Handoff from Aim
    //        // Chamber Command
    //        new FunctionalButton(
    //                () ->
    //                        gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
    //                                && slide.getGoal() == AlphaSlide.Goal.AIM)
    //                .whenPressed(
    //                        handoffCommand
    //                                .get()
    //                                .andThen(new WaitCommand(200))
    //                                .andThen(new InstantCommand(() -> slide.wristUp()))
    //                                .andThen(new WaitCommand(200))
    //                                .andThen(
    //                                        new ParallelCommandGroup(
    //                                                new InstantCommand(() ->
    // lift.setGoal(AlphaLift.Goal.PRE_HANG)),
    //                                                new WaitUntilCommand(() ->
    // lift.getCurrentPosition() > 200)
    //                                                        .andThen(new
    // InstantCommand(liftClaw::upLiftArm)))),
    //                        false);
    //
    //        new FunctionalButton(
    //                () ->
    //                        gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
    //                                && lift.getGoal() == AlphaLift.Goal.HANG)
    //                .whenPressed(new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG)));
    //
    //        new FunctionalButton(
    //                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) &&
    // isPureHandoffCompelte)
    //                .whenPressed(
    //                        new InstantCommand(() -> slide.wristUp())
    //                                .andThen(new WaitCommand(200))
    //                                .andThen(
    //                                        new ParallelCommandGroup(
    //                                                new InstantCommand(() ->
    // lift.setGoal(AlphaLift.Goal.PRE_HANG)),
    //                                                new WaitUntilCommand(() ->
    // lift.getCurrentPosition() > 200)
    //                                                        .andThen(new
    // InstantCommand(liftClaw::upLiftArm)))),
    //                        false);

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    && lift.getGoal() == Lift.Goal.STOW)
        .whenHeld(lift.manualResetCommand());

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                    && slide.getGoal() != AlphaSlide.Goal.HANDOFF)
        .whenPressed(
                new InstantCommand(slide::forwardSlideExtension));

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
  public void run(){
    CommandScheduler.getInstance().run();
    lift.periodicTest();
  }
}