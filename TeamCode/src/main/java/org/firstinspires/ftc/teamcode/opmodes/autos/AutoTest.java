package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
@Autonomous(name = "Test Auto", group = "Autos")
public class AutoTest extends LinearOpMode {

  LiftClaw liftClaw;
  Lift lift;
  SlideSuperStucture slide;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

  public static double xValue4 = 8;
  public static double yValue4 = 16;
  public static double heading4 = 5;

  TrajectorySequence trajs1 =
          TrajectoryManager.trajectorySequenceBuilder(startPose)
                  .lineToLinearHeading(new Pose2d(xValue4, yValue4, Math.toRadians(heading4)))
                  .build();

  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    waitForStart();

    // spotless:off
        CommandScheduler.getInstance()
                .schedule(
                        followTrajectory(drive, trajs1),
                        new WaitCommand(500),
                        new InstantCommand(slide::forwardSlideExtension).alongWith(
                                slide.setServoPosCommand(SlideSuperStucture.TurnServo.DEG_08)
                        ),
                        new WaitCommand(1000),
                        slide.grabCommand(),
                        new WaitCommand(500),
                        slide.handoffCommand()
                );

        // spotless:on

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
    }
  }
}
