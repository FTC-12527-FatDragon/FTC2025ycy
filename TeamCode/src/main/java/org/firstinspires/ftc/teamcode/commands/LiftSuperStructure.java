package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;

public class LiftSuperStructure {
  public final Lift lift;
  public final LiftClaw claw;

  public LiftSuperStructure(Lift lift, LiftClaw claw) {
    this.lift = lift;
    this.claw = claw;
  }

  public Command toHangCommand() {
    return new SequentialCommandGroup(new InstantCommand());
  }

  public Command toNormalCommand() {
    return new ParallelCommandGroup(
        lift.setGoalCommand(Lift.Goal.BASKET)
            .andThen(new InstantCommand(claw::upLiftArm)));
  }
}
