package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
    return new ConditionalCommand(
            new SequentialCommandGroup(new InstantCommand(liftClaw::closeClaw),
            new WaitCommand(300),
            new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG))
                    .alongWith(new InstantCommand(liftClaw::chamberWrist))
                    .andThen(new InstantCommand(liftClaw::chamberLiftArm)))
            ,
            new SequentialCommandGroup(new InstantCommand(liftClaw::closeClaw),
                    new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG))
                            .alongWith(new InstantCommand(liftClaw::chamberWrist))
                            .andThen(new InstantCommand(liftClaw::chamberLiftArm)))
            ,
            liftClaw::getClawStatus
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
        new InstantCommand(liftClaw::initialize),
        new InstantCommand(liftClaw::stowWrist),
        new InstantCommand(liftClaw::foldLiftArm),
        new InstantCommand(liftClaw::closeClaw),
        new InstantCommand(slide::initialize));
  }

  public static Command initializeForce(AlphaLiftClaw liftClaw, AlphaSlide slide) {
    return new ParallelCommandGroup(
            new InstantCommand(liftClaw::initialize),
            new InstantCommand(liftClaw::stowWrist),
            new InstantCommand(liftClaw::foldLiftArm),
            new InstantCommand(liftClaw::closeClaw),
            slide.aimCommand());
  }

  public static Command autoFinish(AlphaLiftClaw liftClaw, AlphaLift lift, AlphaSlide slide) {
    return new SequentialCommandGroup(
        initialize(liftClaw, slide).andThen(slide.aimCommand()).andThen(new WaitCommand(100)));
  }
}
