package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opmodes.TXBetaBotSolo;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config
public class AutoCommand {
  public static long handoff_slide2LiftCloseDelayMs = 0;
  public static long handoff_liftClose2OpenIntakeDelayMs = 50;
  public static Command upLiftToBasket(Lift lift, LiftClaw liftClaw) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 300)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public static Command followTrajectory(
      SampleMecanumDrive drive, TrajectorySequence trajectorySequence) {
    return new AutoDriveCommand(drive, trajectorySequence);
  }

  public static Command stowArmFromBasket(Lift lift, LiftClaw liftClaw) {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::openClaw),
        new WaitCommand(100),
        new InstantCommand(liftClaw::foldLiftArm),
        new WaitCommand(200),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public static Command slowHandoff(SlideSuperStucture slide, LiftClaw liftClaw) {
    return slide
        .slowHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  public static Command fastHandoff(SlideSuperStucture slide, LiftClaw liftClaw) {
    return slide
        .fastHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  public static Command upLiftToChamber(Lift lift, LiftClaw liftClaw) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 100)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public static Command handoffAndLiftToChamber(
      Lift lift, LiftClaw liftClaw, SlideSuperStucture slide) {
    return fastHandoff(slide, liftClaw)
        .andThen(new WaitCommand(150))
        .andThen(new InstantCommand(slide::slideArmDown))
        .andThen(new WaitCommand(150))
        .andThen(upLiftToChamber(lift, liftClaw));
  }

  public static Command hangAndStowLift(Lift lift, LiftClaw liftClaw, SlideSuperStucture slide) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG))
            .alongWith(new InstantCommand(slide::slideArmDown)),
        new ParallelDeadlineGroup(new WaitCommand(500), new WaitUntilCommand(() -> lift.atHome(10)))
            .andThen(new WaitCommand(100).deadlineWith(lift.manualResetCommand()))
            .andThen(new InstantCommand(liftClaw::openClaw)),
        new WaitCommand(200),
        new InstantCommand(liftClaw::foldLiftArm),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public static Command initialize(LiftClaw liftClaw, SlideSuperStucture slide) {
    return new ParallelCommandGroup(
        //        new InstantCommand(slide::forwardSlideExtension),
        new InstantCommand(liftClaw::closeClaw),
        new InstantCommand(slide::slideArmUp),
        new InstantCommand(slide::wristUp),
        new InstantCommand(slide::openIntakeClaw));
  }

  public static Command autoFinish(
      SampleMecanumDrive drive, LiftClaw liftClaw, Lift lift, SlideSuperStucture slide) {
    return new ParallelCommandGroup(
        slide.aimCommand(),
        // TODO: needs discussion
        slide.manualResetCommand().withTimeout(1000), // interruptOn(slide::atHome),
        // lift.resetCommand().interruptOn(() -> lift.atHome(3)),
        lift.manualResetCommand().withTimeout(1000),
        new InstantCommand(liftClaw::openClaw),
        new InstantCommand(() -> TXBetaBotSolo.autoEndPose = drive.getPoseEstimate()));
  }
}
