package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

public class TeleopDriveCommand extends CommandBase {
  private final MecanumDrive drive;
  private final DoubleSupplier forward;
  private final DoubleSupplier rotate;
  private final DoubleSupplier fun;
  private final BooleanSupplier shouldReset;
  private final BooleanSupplier shouldSlow;

  public TeleopDriveCommand(
      MecanumDrive drive,
      DoubleSupplier forward,
      DoubleSupplier fun,
      DoubleSupplier rotate,
      BooleanSupplier shouldReset,
      BooleanSupplier shouldSlow) {
    this.drive = drive;
    this.forward = forward;
    this.rotate = rotate;
    this.fun = fun;
    this.shouldReset = shouldReset;
    this.shouldSlow = shouldSlow;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.reset(Math.toRadians(180));
  }

  @Override
  public void execute() {
    if (shouldReset.getAsBoolean()) {
      drive.reset(Math.toRadians(0));
    }

    double forwardValue = forward.getAsDouble();
    double funValue = fun.getAsDouble();
    double rotateValue = rotate.getAsDouble();

    if (shouldSlow.getAsBoolean()) {
      forwardValue *= 0.3;
      funValue *= 0.3;
      rotateValue *= 0.3;
    }
    drive.moveRobotFieldRelative(forwardValue, funValue, rotateValue);
  }
}
