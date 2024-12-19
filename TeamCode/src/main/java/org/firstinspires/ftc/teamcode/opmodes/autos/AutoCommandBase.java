package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opmodes.TXBetaBotSolo;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config
public abstract class AutoCommandBase extends LinearOpMode {
  protected LiftClaw liftClaw;
  protected Lift lift;
  protected SlideSuperStucture slide;
  protected Pose2d startPose;
  protected SampleMecanumDrive drive;

  public static long handoff_slide2LiftCloseDelayMs = 0;
  public static long handoff_liftClose2OpenIntakeDelayMs = 50;

  public Command upLiftToBasket() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 300)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public Command followTrajectory(TrajectorySequence trajectorySequence) {
    return new AutoDriveCommand(drive, trajectorySequence);
  }

  public Command stowArmFromBasket() {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::openClaw),
        new WaitCommand(100),
        new InstantCommand(liftClaw::foldLiftArm),
        new WaitCommand(200),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public Command slowHandoff() {
    return slowHandoff(slide, liftClaw);
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

  public Command fastHandoff() {
    return slide
        .fastHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  public Command upLiftToChamber() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 90)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public Command handoffAndLiftToChamber() {
    return fastHandoff()
        .andThen(new WaitCommand(150))
        .andThen(new InstantCommand(slide::slideArmDown))
        .andThen(new WaitCommand(150))
        .andThen(upLiftToChamber());
  }

  public Command hangAndStowLift() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG))
            .alongWith(new InstantCommand(slide::slideArmDown)),
        new ParallelDeadlineGroup(new WaitCommand(100), new WaitUntilCommand(() -> lift.atHome(10)))
            .andThen(new WaitCommand(200).deadlineWith(lift.manualResetCommand()))
            .andThen(
                new InstantCommand(() -> drive.setWeightedDrivePower(new Pose2d(1, 0, 0)))
                    .andThen(new WaitCommand(50))
                    .andThen(
                        new InstantCommand(() -> drive.setWeightedDrivePower(new Pose2d(0, 0, 0)))))
            .andThen(new InstantCommand(liftClaw::openClaw)),
        new WaitCommand(50),
        new InstantCommand(liftClaw::foldLiftArm),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public Command initialize() {
    return new ParallelCommandGroup(
        //        new InstantCommand(slide::forwardSlideExtension),
        new InstantCommand(liftClaw::closeClaw),
        new InstantCommand(slide::slideArmUp),
        new InstantCommand(slide::wristUp),
        new InstantCommand(slide::openIntakeClaw));
  }

  public Command autoFinish() {
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
