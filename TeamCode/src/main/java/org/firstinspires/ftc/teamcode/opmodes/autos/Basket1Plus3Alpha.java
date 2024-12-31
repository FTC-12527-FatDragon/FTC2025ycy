package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.handoff;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.stowArmFromBasket;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.upLiftToBasket;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.autoFinish;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.handoff;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.initialize;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.stowArmFromBasket;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.upLiftToBasket;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
@Autonomous(name = "Alpha Basket 1+3", group = "Autos")
public class Basket1Plus3Alpha extends LinearOpMode {
  // For Basket Scoring
  public static double xValue1 = 10;
  public static double yValue1 = 20;
  public static double heading1 = -45;

  // The rightmost sample
  public static double xValue2 = 25;
  public static double yValue2 = 9.5;
  public static double heading2 = 0;

  // The middle sample
  public static double xValue3 = 0;
  public static double yValue3 = 0;
  public static double heading3 = 0;

  // The leftmost sample
  public static double xValue4 = 0;
  public static double yValue4 = 0;
  public static double heading4 = 0;

  // The Ascent zone
  public static double xValue5 = 0;
  public static double yValue5 = 0;
  public static double heading5 = 0;

  AlphaLiftClaw liftClaw;
  Lift lift;
  AlphaSlide slide;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

  // Start to Basket
  TrajectorySequence trajs1 =
      TrajectoryManager.trajectorySequenceBuilder(startPose)
          .lineToLinearHeading(new Pose2d(xValue1, yValue1, Math.toRadians(heading1)))
          .build();

  // Basket to the rightmost sample
  TrajectorySequence trajs2 =
      TrajectoryManager.trajectorySequenceBuilder(trajs1.end())
          .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
          .build();

  // rightmost sample to basket
  TrajectorySequence trajs3 =
      TrajectoryManager.trajectorySequenceBuilder(trajs2.end())
          .lineToLinearHeading(new Pose2d(xValue1, yValue1, Math.toRadians(heading1)))
          .build();

  // basket to middle sample
  TrajectorySequence trajs4 =
      TrajectoryManager.trajectorySequenceBuilder(trajs3.end())
          .lineToLinearHeading(new Pose2d(xValue3, yValue3, Math.toRadians(heading3)))
          .build();

  // middle sample to basket
  TrajectorySequence trajs5 =
      TrajectoryManager.trajectorySequenceBuilder(trajs4.end())
          .lineToLinearHeading(new Pose2d(xValue1, yValue1, Math.toRadians(heading1)))
          .build();

  // basket to leftmost sample
  TrajectorySequence trajs6 =
      TrajectoryManager.trajectorySequenceBuilder(trajs5.end())
          .lineToLinearHeading(new Pose2d(xValue4, yValue4, Math.toRadians(heading4)))
          .build();

  // leftmost sample to basket
  TrajectorySequence trajs7 =
      TrajectoryManager.trajectorySequenceBuilder(trajs6.end())
          .lineToLinearHeading(new Pose2d(xValue1, yValue1, Math.toRadians(heading1)))
          .build();

  // basket to ascent zone
  TrajectorySequence trajs8 =
      TrajectoryManager.trajectorySequenceBuilder(trajs7.end())
          .lineToLinearHeading(new Pose2d(xValue5, yValue5, Math.toRadians(heading5)))
          .build();

  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    // spotless:off
    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                initialize(liftClaw, slide),

                new AutoDriveCommand(drive, trajs1).alongWith(upLiftToBasket(lift, liftClaw)),
                    stowArmFromBasket(lift, liftClaw),

                new AutoDriveCommand(drive, trajs2),
                slide.grabCommand(),
                new AutoDriveCommand(drive, trajs3)
                    .alongWith(handoff(slide, liftClaw).andThen(upLiftToBasket(lift, liftClaw))),
                stowArmFromBasket(lift, liftClaw),

                new AutoDriveCommand(drive, trajs4).alongWith(slide.aimCommand()),
                slide.grabCommand(),
                new AutoDriveCommand(drive, trajs5)
                    .alongWith(handoff(slide, liftClaw).andThen(upLiftToBasket(lift, liftClaw))),
                    stowArmFromBasket(lift, liftClaw),

                new AutoDriveCommand(drive, trajs6).alongWith(slide.aimCommand()),
                slide.grabCommand(),
                new AutoDriveCommand(drive, trajs7)
                    .alongWith(handoff(slide, liftClaw).andThen(upLiftToBasket(lift, liftClaw))),
                    stowArmFromBasket(lift, liftClaw),

                new AutoDriveCommand(drive, trajs8).alongWith(autoFinish(liftClaw, lift, slide))));

    // spotless:on
    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
    }
  }
}
