package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config
@Autonomous(name = "TestTraj", group = "Autos")
public class TestTraj extends LinearOpMode {
  AlphaLiftClaw liftClaw;
  AlphaLift lift;
  AlphaSlide slide;

  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    lift = new AlphaLift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    TrajectorySequence trajectory0 =
        drive
            .trajectorySequenceBuilder(new Pose2d(24.43, -64.95, Math.toRadians(90.00)))
            .lineTo(new Vector2d(6.49, -30.74))
            .build(); // 1+0

    TrajectorySequence trajectory1 =
        drive
            .trajectorySequenceBuilder(new Pose2d(6.49, -30.74, Math.toRadians(90.00)))
            .splineTo(new Vector2d(25.71, -39.80), Math.toRadians(-0.32))
            .splineTo(new Vector2d(34.68, -26.03), Math.toRadians(75.96))
            .splineToSplineHeading(
                new Pose2d(47.49, -10.97, Math.toRadians(90.00)), Math.toRadians(-90.00))
            .lineToSplineHeading(new Pose2d(47.33, -59.03, Math.toRadians(90.00)))
            .splineToSplineHeading(
                new Pose2d(53.58, -8.73, Math.toRadians(90.00)), Math.toRadians(77.37))
            .lineToSplineHeading(new Pose2d(54.54, -59.83, Math.toRadians(90.00)))
            .build();

    TrajectorySequence trajectory2 =
        drive
            .trajectorySequenceBuilder(new Pose2d(59.67, -55.98, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(36.60, -60.31, Math.toRadians(90.00)))
            .build(); // push end to grab

    TrajectorySequence trajectory3 =
        drive
            .trajectorySequenceBuilder(new Pose2d(36.60, -60.31, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(6.49, -30.74, Math.toRadians(90.00)))
            .build(); // grab to chamber

    drive.setPoseEstimate(trajectory0.start());

    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                initialize(liftClaw, slide), new AutoDriveCommand(drive, trajectory1)));

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
    }
  }
}
