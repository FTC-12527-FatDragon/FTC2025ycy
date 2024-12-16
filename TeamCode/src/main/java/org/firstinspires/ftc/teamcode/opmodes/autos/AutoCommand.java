package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;

public class AutoCommand {
  public static Command upLiftToBasket(AlphaLift lift, AlphaLiftClaw liftClaw) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 400)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public static Command stowArmFromBasket(AlphaLift lift, AlphaLiftClaw liftClaw) {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::openClaw),
        new WaitCommand(100),
        new InstantCommand(liftClaw::foldLiftArm),
        new WaitCommand(200),
        new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.STOW)));
  }

  public static Command handoff(AlphaSlide slide, AlphaLiftClaw liftClaw) {
    return slide
        .handoffCommand()
        .alongWith(new InstantCommand(liftClaw::openClaw))
        .andThen(new WaitCommand(600))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(200))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  public static Command grabToPreHang(AlphaLift lift, AlphaLiftClaw liftClaw) {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::closeClaw)
            .andThen(new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG))
                    .alongWith(new InstantCommand(liftClaw::chamberWrist))
                    .andThen(new InstantCommand(liftClaw::chamberLiftArm))
            )
    );
  }

  public static Command upToChamber(AlphaLift lift) {
    return new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.HANG));
  }

  public static Command chamberToGrab(AlphaLift lift, AlphaLiftClaw liftClaw) {
    return new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.GRAB))
            .alongWith(new InstantCommand(liftClaw::openClaw))
            .andThen(new InstantCommand(liftClaw::grabWrist))
            .alongWith(new InstantCommand(liftClaw::grabLiftArm));
  }

  public static Command initialize(AlphaLiftClaw liftClaw, AlphaSlide slide) {
    return new ParallelCommandGroup(
        new InstantCommand(slide::forwardSlideExtension),
        new InstantCommand(liftClaw::closeClaw),
        new InstantCommand(slide::slideArmUp),
        new InstantCommand(slide::wristUp),
        new InstantCommand(slide::openIntakeClaw));
  }

  public static Command autoFinish(AlphaLiftClaw liftClaw, AlphaLift lift, AlphaSlide slide) {
    return new ParallelCommandGroup(
        initialize(liftClaw, slide),
        lift.resetCommand().withTimeout(300),
        new InstantCommand(liftClaw::openClaw));
  }
}
