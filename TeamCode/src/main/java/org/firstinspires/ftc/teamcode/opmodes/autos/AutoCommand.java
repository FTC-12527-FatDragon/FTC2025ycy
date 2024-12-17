package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

public class AutoCommand {
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

  public static Command handoff(SlideSuperStucture slide, LiftClaw liftClaw) {
    return slide
        .slowHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(50))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(200))
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
    return handoff(slide, liftClaw)
        .andThen(new WaitCommand(200))
        .andThen(new InstantCommand(slide::wristUp))
        .andThen(new WaitCommand(300))
        .andThen(upLiftToChamber(lift, liftClaw));
  }

  public static Command hangAndStowLift(Lift lift, LiftClaw liftClaw, SlideSuperStucture slide) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG))
            .alongWith(new InstantCommand(slide::slideArmDown)),
        new ParallelDeadlineGroup(new WaitCommand(500), new WaitUntilCommand(() -> lift.atHome(10)))
            .andThen(new WaitCommand(150))
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

  public static Command autoFinish(LiftClaw liftClaw, Lift lift, SlideSuperStucture slide) {
    return new ParallelCommandGroup(
        slide.aimCommand(),
        // TODO: needs discussion
        slide.manualResetCommand().withTimeout(1000), // interruptOn(slide::atHome),
        // lift.resetCommand().interruptOn(() -> lift.atHome(3)),
        lift.manualResetCommand().withTimeout(1000),
        new InstantCommand(liftClaw::openClaw));
  }
}
