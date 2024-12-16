package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

import java.util.Collections;
import java.util.Set;

@Config
@Autonomous(name = "Basket 1+3", group = "Autos")
public class Basket1Plus3 extends LinearOpMode {
  // For Basket Scoring
  public static double xValue1 = 10;
  public static double yValue1 = 16;
  public static double heading1 = -45;

  // The right sample
  public static double xValue2 = 23.5;
  public static double yValue2 = 9.5;
  public static double heading2 = 0;

  // The middle sample
  public static double xValue3 = 23.5;
  public static double yValue3 = 20;
  public static double heading3 = 0;

  // The left sample
  public static double xValue4 = 11;
  public static double yValue4 = 20;
  public static double heading4 = 20;

  public static long basketWaitMs = 500;

  //  // Ascent zone
  //  public static double xValue5 = 60;
  //  public static double yValue5 = -16;
  //  public static double heading5 = 0;

  LiftClaw liftClaw;
  Lift lift;
  SlideSuperStucture slide;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

  // Start to Basket
  TrajectorySequence trajs1 =
      TrajectoryManager.trajectorySequenceBuilder(startPose)
          .splineToLinearHeading(new Pose2d(xValue1, yValue1, Math.toRadians(heading1)), heading1)
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

  //  // basket to ascent zone
  //  TrajectorySequence trajs8 =
  //      TrajectoryManager.trajectorySequenceBuilder(trajs7.end())
  //          .lineToLinearHeading(new Pose2d(xValue5, yValue5, Math.toRadians(heading5)))
  //          .build();

  public Command wait(SampleMecanumDrive drive, long ms){
    return new ParallelDeadlineGroup(new WaitCommand(ms), new FunctionalCommand(
            () -> {},
            drive::update,
            (b) -> {},
            () -> {return drive.isBusy() && !isStopRequested();}
    ));
  }

  @Override
  public void runOpMode() throws InterruptedException {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    liftClaw.closeClaw();

    waitForStart();

    // spotless:off
    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                  drive.setPoseEstimate(trajs1.start());
                  slide.slideOpenloop(-0.5);
                }),
                slide.aimCommand().beforeStarting(liftClaw::closeClaw),

                followTrajectory(drive, trajs1).alongWith(upLiftToBasket(lift, liftClaw)),
                wait(drive, 200),
                stowArmFromBasket(lift, liftClaw),

                followTrajectory(drive, trajs2).alongWith(slide.aimCommand()),
                slide.grabCommand(),
                followTrajectory(drive, trajs3)
                    .alongWith(handoff(slide, liftClaw).andThen(upLiftToBasket(lift, liftClaw))),
                wait(drive, basketWaitMs),
                stowArmFromBasket(lift, liftClaw),

                followTrajectory(drive, trajs4).alongWith(slide.aimCommand()),
                slide.grabCommand(),
                followTrajectory(drive, trajs5)
                    .alongWith(handoff(slide, liftClaw).andThen(upLiftToBasket(lift, liftClaw))),
                wait(drive, basketWaitMs),
                stowArmFromBasket(lift, liftClaw),

                followTrajectory(drive, trajs6).alongWith(slide.aimCommand()),
                wait(drive, 300),
                new InstantCommand(() -> slide.forwardSlideExtension(440)).alongWith(
                    slide.setServoPosCommand(SlideSuperStucture.TurnServo.DEG_0)),
                wait(drive, 500),
                slide.grabCommand(),
                wait(drive, 300),
                handoff(slide, liftClaw),
                wait(drive, 300),
                followTrajectory(drive, trajs7),
                upLiftToBasket(lift, liftClaw),
                wait(drive, basketWaitMs),
                stowArmFromBasket(lift, liftClaw),
                wait(drive, 2000),
                autoFinish(liftClaw, lift, slide)
            ));

    // spotless:off

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
      lift.periodicTest();
    }
  }
}
