package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;

public class AlphaAutoCommand {
    public static Command alphaUpLiftToBasket(AlphaLift lift, AlphaLiftClaw liftClaw) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.BASKET)),
                new WaitUntilCommand(() -> lift.getCurrentPosition() > 400)
                        .andThen(new InstantCommand(liftClaw::upLiftArm)));
    }

    public static Command alphaStowArmFromBasket(AlphaLift lift, AlphaLiftClaw liftClaw) {
        return new SequentialCommandGroup(
                new InstantCommand(liftClaw::openClaw),
                new WaitCommand(100),
                new InstantCommand(liftClaw::foldLiftArm),
                new WaitCommand(200),
                new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.STOW)));
    }

    public static Command alphaHandoff(AlphaSlide slide, AlphaLiftClaw liftClaw) {
        return slide
                .handoffCommand()
                .alongWith(new InstantCommand(liftClaw::openClaw))
                .andThen(new WaitCommand(600))
                .andThen(new InstantCommand(liftClaw::closeClaw))
                .andThen(new WaitCommand(200))
                .andThen(new InstantCommand(slide::openIntakeClaw));
    }

    public static Command alphaUpLiftToChamber(AlphaLift lift, AlphaLiftClaw liftClaw) {
        return new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG))
                .alongWith(new InstantCommand(liftClaw::chamberWrist))
                .andThen(new InstantCommand(liftClaw::chamberLiftArm));
    }

    public static Command alphaHangAndStowLift(AlphaLift lift, AlphaLiftClaw liftClaw, AlphaSlide slide) {
        return new SequentialCommandGroup(
                new InstantCommand(liftClaw::openClaw),
                new WaitCommand(300),
                new InstantCommand(liftClaw::foldLiftArm),
                new WaitCommand(500),
                new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.STOW)));
    }

    public static Command hangAndStowLift(AlphaLift lift, AlphaLiftClaw liftClaw) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.HANG)),
                new WaitCommand(200),
        new InstantCommand(liftClaw::openClaw)
                .andThen(new InstantCommand(liftClaw::foldLiftArm))
                .alongWith(new InstantCommand(liftClaw::basketWrist))
                .andThen(new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.STOW))));
    }
}
