package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.subsystems.AlphaSlide.grabTimeout;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class AutoCommand {
  public static long lift2BasketTimeout = 800;
  public static long slideServoResponseTime = 600;
  public static long basketTimeout = 600;
  public static long handoffTimeout = 100;
  public static long tempTimeout = 1000;
  public static Command upLiftToBasket(Lift lift, AlphaLiftClaw liftClaw) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 400)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public static Command stowArmFromBasket(Lift lift, AlphaLiftClaw liftClaw) {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::openClaw),
        new WaitCommand(100),
        liftClaw.foldLiftArmCommand(),
        lift.setGoalCommand(Lift.Goal.STOW));
  }

  public static Command handoff(AlphaSlide slide, AlphaLiftClaw liftClaw) {
    return slide
        .handoffCommand()
        .alongWith(new InstantCommand(liftClaw::openClaw))
        .andThen(new WaitCommand(600))
        .andThen(liftClaw.closeClawCommand())
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  public static Command toPreHang(Lift lift, AlphaLiftClaw liftClaw) {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::chamberWrist),
        new InstantCommand(liftClaw::chamberLiftArm),
        lift.setGoalCommand(Lift.Goal.PRE_HANG)
    );
  }

  public static Command grabToPreHang(Lift lift, AlphaLiftClaw liftClaw){
    return liftClaw.closeClawCommand().andThen(toPreHang(lift, liftClaw));
  }

  public static Command upToChamber(Lift lift) {
    return new WaitCommand(500).deadlineWith(lift.setGoalCommand(Lift.Goal.HANG));
  }

  public static Command chamberOpenClaw(AlphaLiftClaw liftClaw) {
    return new InstantCommand(liftClaw::openClaw);
  }

  public static Command chamberToGrab(Lift lift, AlphaLiftClaw liftClaw) {
    return new InstantCommand(liftClaw::openClaw)
        .andThen(new InstantCommand(liftClaw::grabWrist))
        .andThen(new InstantCommand(liftClaw::grabLiftArm))
            .andThen(lift.setGoalCommand(Lift.Goal.GRAB, false));
  }

  public static Command initialize(AlphaLiftClaw liftClaw, AlphaSlide slide) {
    return new ParallelCommandGroup(
        new InstantCommand(liftClaw::initialize),
        new InstantCommand(liftClaw::stowWrist),
        liftClaw.foldLiftArmCommand(0),
        liftClaw.closeClawCommand(0),
        new InstantCommand(slide::initialize));
  }


  public static Command liftToBasket(AlphaLiftClaw liftClaw, Lift lift) {
    return new SequentialCommandGroup(
            new WaitCommand(lift2BasketTimeout).deadlineWith(lift.setGoalCommand(Lift.Goal.BASKET)),
            new InstantCommand(liftClaw::upLiftArm)
                    .alongWith(new InstantCommand(liftClaw::basketWrist)),
            new WaitCommand(basketTimeout),
            new InstantCommand(liftClaw::openClaw)

    );
  }


  public static Command liftBack(AlphaLiftClaw liftClaw, Lift lift) {
    return new ParallelCommandGroup(
            liftClaw.foldLiftArmCommand(),
            new WaitCommand(tempTimeout),
            new InstantCommand(liftClaw::stowWrist),
            new WaitCommand(basketTimeout),
            new WaitCommand(lift2BasketTimeout).deadlineWith(lift.setGoalCommand(Lift.Goal.STOW))
    );
  }


  public static Command grabAndBack(AlphaLiftClaw liftClaw, AlphaSlide slide) {
    return new SequentialCommandGroup(
            new InstantCommand(slide::autoForwardSlideExtension),
            new WaitCommand(slideServoResponseTime),
            slide.autoGrabCommand(),
            new InstantCommand(slide::autoBackSlideExtension),
            new WaitCommand(slideServoResponseTime),
            liftClaw.closeClawCommand(),
            new WaitCommand(handoffTimeout),
            slide.autoOpenIntakeClaw()
    );
  }

  public static Command initializeForce(AlphaLiftClaw liftClaw, AlphaSlide slide) {
    return new ParallelCommandGroup(
        new InstantCommand(liftClaw::initialize),
        new InstantCommand(liftClaw::stowWrist),
        liftClaw.foldLiftArmCommand(0),
        new InstantCommand(liftClaw::closeClawCommand),
        slide.aimCommand());
  }

  public static Command autoFinish(AlphaLiftClaw liftClaw, Lift lift, AlphaSlide slide) {
    return new SequentialCommandGroup(
        initialize(liftClaw, slide).andThen(slide.aimCommand()).andThen(new WaitCommand(100)));
  }
}
