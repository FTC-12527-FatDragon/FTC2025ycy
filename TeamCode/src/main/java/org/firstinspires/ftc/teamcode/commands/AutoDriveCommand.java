package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

public class AutoDriveCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private final Trajectory trajectory;
  private final TrajectorySequence trajectorySequence;
//  private final Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

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

//  private final ElapsedTime timer = new ElapsedTime();

  @Override
  public void initialize() {
//    long lasttime = timer.time(TimeUnit.MILLISECONDS);
    if (trajectory != null) {
      //      drive.setPoseEstimate(trajectory.start());
      drive.followTrajectoryAsync(trajectory);
    } else if (trajectorySequence != null) {
      //      drive.setPoseEstimate(trajectorySequence.start());
      drive.followTrajectorySequenceAsync(trajectorySequence);
    } else throw new IllegalArgumentException("No Trajectory or TrajectorySequence provide");
//    telemetry.addData("AutoDriveTime", timer.time(TimeUnit.MILLISECONDS)-lasttime);
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
