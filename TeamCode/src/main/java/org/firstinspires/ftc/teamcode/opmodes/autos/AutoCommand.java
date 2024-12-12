package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;

public class AutoCommand {
  public static Command upLiftToBasket(Lift lift, LiftClaw liftClaw) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 400)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
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
        .handoffCommand()
        .alongWith(new InstantCommand(liftClaw::openClaw))
        .andThen(new WaitCommand(600))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(200))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  public static Command upLiftToChamber(Lift lift, LiftClaw liftClaw) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 200)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public static Command hangAndStowLift(Lift lift, LiftClaw liftClaw, SlideSuperStucture slide) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG)),
        new WaitCommand(200),
        new InstantCommand(slide::slideArmDown)
            .andThen(new WaitCommand(100))
            .andThen(new InstantCommand(() -> slide.setGoal(SlideSuperStucture.Goal.AIM))),
        new InstantCommand(liftClaw::openClaw),
        new WaitCommand(100),
        new InstantCommand(liftClaw::foldLiftArm),
        new WaitCommand(500),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public static Command initialize(LiftClaw liftClaw, SlideSuperStucture slide) {
    return new ParallelCommandGroup(
        new InstantCommand(slide::forwardSlideExtension),
        new InstantCommand(liftClaw::closeClaw),
        new InstantCommand(slide::slideArmUp),
        new InstantCommand(slide::wristUp),
        new InstantCommand(slide::openIntakeClaw));
  }

  public static Command autoFinish(LiftClaw liftClaw, Lift lift, SlideSuperStucture slide) {
    return new ParallelCommandGroup(
        slide.aimCommand(),
        lift.resetCommand().withTimeout(300),
        new InstantCommand(liftClaw::openClaw));
  }
}
