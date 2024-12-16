package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@TeleOp(name = "Solo")
public class TXBetaBotSolo extends CommandOpMode {
  private GamepadEx gamepadEx1;
  private Lift lift;
  private LiftClaw liftClaw;
  private SlideSuperStucture slide;
  private SampleMecanumDrive drive;

  private boolean isPureHandoffCompelte = false;

  @Override
  public void initialize() {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    gamepadEx1 = new GamepadEx(gamepad1);

    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);
    drive = new SampleMecanumDrive(hardwareMap);

    // Teleop Drive Command
    drive.setDefaultCommand(
        new TeleopDriveCommand(
            drive,
            () -> -gamepadEx1.getLeftY(),
            () -> gamepadEx1.getLeftX(),
            () -> -gamepadEx1.getRightX(),
            () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON),
            () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)));

    // Basket Up Command
    gamepadEx1
        .getGamepadButton(GamepadKeys.Button.X)
        .whenPressed(
            new ParallelCommandGroup(
                new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
                new WaitUntilCommand(() -> lift.getCurrentPosition() > 300)
                    .andThen(new InstantCommand(liftClaw::upLiftArm))));

    // Basket Drop and Back
    gamepadEx1
        .getGamepadButton(GamepadKeys.Button.B)
        .whenPressed(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(() -> slide.slideArmDown())
                        .andThen(
                            new WaitCommand(100),
                            new InstantCommand(() -> slide.setGoal(SlideSuperStucture.Goal.AIM)),
                            new InstantCommand(liftClaw::openClaw),
                            new WaitCommand(100),
                            new InstantCommand(liftClaw::foldLiftArm)),
                    new InstantCommand(liftClaw::openClaw)
                        .andThen(
                            new WaitCommand(100),
                            new InstantCommand(liftClaw::foldLiftArm),
                            new WaitCommand(200)),
                    () -> lift.getGoal() == Lift.Goal.HANG),
                new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)),
                new InstantCommand(() -> isPureHandoffCompelte = false)));

    // Aim
    gamepadEx1
        .getGamepadButton(GamepadKeys.Button.Y)
        .whenPressed(
            slide.aimCommand().alongWith(new InstantCommand(() -> isPureHandoffCompelte = false)),
            false);

    // Grab when aim
    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.A)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM)
        .whenPressed(slide.grabCommand(), false);

    // Pure Handoff
    Supplier<Command> handoffCommand =
        () ->
            slide
                .handoffCommand()
                .beforeStarting(liftClaw::openClaw)
                .andThen(new WaitCommand(50))
                .andThen(new InstantCommand(() -> liftClaw.closeClaw()))
                .andThen(new WaitCommand(200))
                .andThen(new InstantCommand(() -> slide.openIntakeClaw()))
                .andThen(new WaitCommand(50))
                .andThen(new InstantCommand(() -> isPureHandoffCompelte = true));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && lift.getGoal() == Lift.Goal.STOW)
        .whenPressed(handoffCommand.get(), false);

    // Handoff from Aim
    // Chamber Command
    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && lift.getGoal() == Lift.Goal.STOW)
        .whenPressed(
            handoffCommand
                .get()
                .andThen(new WaitCommand(200))
                .andThen(new InstantCommand(() -> slide.wristUp()))
                .andThen(new WaitCommand(200))
                .andThen(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
                        new WaitUntilCommand(() -> lift.getCurrentPosition() > 100)
                            .andThen(new InstantCommand(liftClaw::upLiftArm)))),
            false);

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    && lift.getGoal() == Lift.Goal.HANG)
        .whenPressed(new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    && lift.getGoal() == Lift.Goal.STOW
                    && isPureHandoffCompelte)
        .whenPressed(
            new InstantCommand(() -> slide.wristUp())
                .andThen(new WaitCommand(200))
                .andThen(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
                        new WaitUntilCommand(() -> lift.getCurrentPosition() > 100)
                            .andThen(new InstantCommand(liftClaw::upLiftArm)))),
            false);

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                    && lift.getGoal() == Lift.Goal.PRE_HANG)
        .whenPressed(new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG)));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    && lift.getGoal() == Lift.Goal.STOW)
        .whenPressed(lift.manualResetCommand().withTimeout(100));

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM)
        .whenPressed(new InstantCommand(slide::backwardSlideExtension));

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM)
        .whenPressed(new InstantCommand(slide::forwardSlideExtension));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM)
        .whenPressed(new InstantCommand(() -> slide.leftTurnServo()));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM)
        .whenPressed(new InstantCommand(() -> slide.rightTurnServo()));
  }

  @Override
  public void run() {
    lift.periodicTest();
    CommandScheduler.getInstance().run();
    TelemetryPacket packet = new TelemetryPacket();
    packet.fieldOverlay().setStroke("#3F51B5");
    LocalizationTest.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
    FtcDashboard.getInstance().sendTelemetryPacket(packet);
  }

  @Override
  public void reset() {
    CommandScheduler.getInstance().reset();
    slide.setServoController(false);
    liftClaw.setServoController(false);
  }
}
