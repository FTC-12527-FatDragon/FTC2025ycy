package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
@Autonomous(name = "Chamber 1+3", group = "Autos")
public class Chamber1Plus3 extends LinearOpMode {
  public static double chamberSpacing = 4;
  // For Basket Scoring
  public static double xValue1 = 14;
  public static double yValue1 = -32.425;
  public static double heading1 = 90;

  // The rightmost sample
  public static double xValue2 = -19;
  public static double yValue2 = -4.7;
  public static double heading2 = -180;

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

  LiftClaw liftClaw;
  Lift lift;
  SlideSuperStucture slide;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

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
                  .lineToLinearHeading(new Pose2d(xValue1-chamberSpacing, yValue1, Math.toRadians(heading1)))
                  .build();

  // basket to middle sample
  TrajectorySequence trajs4 =
          TrajectoryManager.trajectorySequenceBuilder(trajs3.end())
                  .lineToLinearHeading(new Pose2d(xValue3, yValue3, Math.toRadians(heading3)))
                  .build();

  // middle sample to basket
  TrajectorySequence trajs5 =
          TrajectoryManager.trajectorySequenceBuilder(trajs4.end())
                  .lineToLinearHeading(new Pose2d(xValue1-chamberSpacing*2, yValue1, Math.toRadians(heading1)))
                  .build();

  // basket to leftmost sample
  TrajectorySequence trajs6 =
          TrajectoryManager.trajectorySequenceBuilder(trajs5.end())
                  .lineToLinearHeading(new Pose2d(xValue4, yValue4, Math.toRadians(heading4)))
                  .build();

  // leftmost sample to basket
  TrajectorySequence trajs7 =
          TrajectoryManager.trajectorySequenceBuilder(trajs6.end())
                  .lineToLinearHeading(new Pose2d(xValue1-chamberSpacing*3, yValue1, Math.toRadians(heading1)))
                  .build();

  // basket to ascent zone
  TrajectorySequence trajs8 =
          TrajectoryManager.trajectorySequenceBuilder(trajs7.end())
                  .lineToLinearHeading(new Pose2d(xValue5, yValue5, Math.toRadians(heading5)))
                  .build();

  @Override
  public void runOpMode() throws InterruptedException {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    // Score the first chamber
    // Push all three samples to the observation zone
    // Repeatedly score the high chamber with slightly different
    slide.stow();
    liftClaw.closeClaw();
    liftClaw.foldLiftArm();
    waitForStart();
    CommandScheduler.getInstance()
            .schedule(
                    new SequentialCommandGroup(
                            slide.aimCommand().beforeStarting(liftClaw::closeClaw),
                            followTrajectory(drive, trajs1).alongWith(upLiftToChamber(lift, liftClaw)),
                            hangAndStowLift(lift, liftClaw, slide),
                            followTrajectory(drive, trajs2).alongWith(slide.aimCommand()),
                            slide.grabCommand(),
                            handoffAndLiftToChamber(lift, liftClaw, slide).alongWith(new WaitCommand(2000).andThen(followTrajectory(drive, trajs3))),
                            new WaitCommand(400),
                            hangAndStowLift(lift, liftClaw, slide)
                    ));
    while (opModeIsActive() && !isStopRequested()) {
      lift.periodicTest();
      CommandScheduler.getInstance().run();
    }
  }
}
