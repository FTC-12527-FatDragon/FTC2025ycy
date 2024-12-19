package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import java.security.AuthProvider;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

public class AutoDriveCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private Trajectory trajectory = null;
  private TrajectorySequence trajectorySequence = null;

  public AutoDriveCommand(
      SampleMecanumDrive drive, Optional<Trajectory> traj, Optional<TrajectorySequence> trajs) {
    this.drive = drive;
    if (traj.isPresent() && trajs.isPresent()) {
      throw new IllegalArgumentException("Cannot provide both Trajectory and TrajectorySequence");
    }
    trajectory = traj.orElse(null);
    trajectorySequence = trajs.orElse(null);
  }

  public AutoDriveCommand(SampleMecanumDrive drive, TrajectorySequence trajs) {
    this(drive, Optional.empty(), Optional.of(trajs));
  }

  public AutoDriveCommand(SampleMecanumDrive drive, Trajectory traj) {
    this(drive, Optional.of(traj), Optional.empty());
  }

  @Override
  public void initialize() {
    if (trajectory != null) {
      drive.setPoseEstimate(trajectory.start());
      drive.followTrajectoryAsync(trajectory);
    } else if (trajectorySequence != null) {
      drive.setPoseEstimate(trajectorySequence.start());
      drive.followTrajectorySequence(trajectorySequence);
    } else throw new IllegalArgumentException("No Trajectory or TrajectorySequence provide");
  }

  @Override
  public void execute() {
    drive.update();
  }

  @Override
  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return !drive.isBusy();
  }
}
