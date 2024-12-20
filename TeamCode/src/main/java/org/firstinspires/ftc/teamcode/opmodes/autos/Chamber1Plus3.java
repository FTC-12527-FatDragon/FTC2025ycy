package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
@Autonomous(name = "Chamber 1+3", group = "Autos")
public class Chamber1Plus3 extends AutoCommandBase {
  public static double chamberSpacing = -3;
  public static long handOff2TrajDelay = 300;
  public static long swipeDelay = 900;
  public static double sampleSpacing = 10.5;

  // Chamber hang location
  public static double xValue1 = 7;
  public static double yValue1 = -33;
  public static double heading1 = 90;

  // Grab location
  public static double xValue2 = -4.5;
  public static double yValue2 = -4.5;
  public static double heading2 = -180;

  // The middle sample
  public static double SampleSupplyX = -16;
  public static double SampleSupplyY = -19;
  public static double SampleSupplyHeading = -120;
  public static double SampleSupplyTurnDeg = -110;

  // The leftmost sample
  public static double xValue4 = 0;
  public static double yValue4 = 0;
  public static double heading4 = 0;

  // The Ascent zone
  public static double xValue5 = 0;
  public static double yValue5 = 0;
  public static double heading5 = 0;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

  // Start to Basket
  TrajectorySequence start2Chamber1 =
      TrajectoryManager.trajectorySequenceBuilder(startPose)
          .lineToLinearHeading(new Pose2d(xValue1, yValue1, Math.toRadians(heading1)))
          .build();

  // Basket to the rightmost sample
  TrajectorySequence Chamber12Grab =
      TrajectoryManager.trajectorySequenceBuilder(start2Chamber1.end())
          .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
          .build();

  // rightmost sample to basket
  TrajectorySequence Grab2Chamber2 =
      TrajectoryManager.trajectorySequenceBuilder(Chamber12Grab.end())
          .lineToLinearHeading(
              new Pose2d(xValue1 - chamberSpacing, yValue1, Math.toRadians(heading1)))
          .build();

  // basket to middle sample
  TrajectorySequence Chamber22Grab =
      TrajectoryManager.trajectorySequenceBuilder(Grab2Chamber2.end())
          .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
          .build();

  // middle sample to basket
  TrajectorySequence Grab2Chamber3 =
      TrajectoryManager.trajectorySequenceBuilder(Chamber22Grab.end())
          .lineToLinearHeading(
              new Pose2d(xValue1 - chamberSpacing * 2, yValue1, Math.toRadians(heading1)))
          .build();

  // basket to leftmost sample
  TrajectorySequence Chamber32Grab =
      TrajectoryManager.trajectorySequenceBuilder(Grab2Chamber3.end())
          .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
          .build();

  // leftmost sample to basket
  TrajectorySequence Grab2Chamber4 =
      TrajectoryManager.trajectorySequenceBuilder(Chamber32Grab.end())
          .lineToLinearHeading(
              new Pose2d(xValue1 - chamberSpacing * 3, yValue1, Math.toRadians(heading1)))
          .build();

  // basket to ascent zone
  TrajectorySequence Chamber12Sample1 =
      TrajectoryManager.trajectorySequenceBuilder(start2Chamber1.end())
          .lineToLinearHeading(
              new Pose2d(SampleSupplyX, SampleSupplyY, Math.toRadians(SampleSupplyHeading)))
          .turn(Math.toRadians(SampleSupplyTurnDeg))
          .build();

  TrajectorySequence Sample12Sample2 =
      TrajectoryManager.trajectorySequenceBuilder(Chamber12Sample1.end())
          .lineToLinearHeading(
              new Pose2d(
                  SampleSupplyX - sampleSpacing,
                  SampleSupplyY,
                  Math.toRadians(SampleSupplyHeading)))
          .turn(Math.toRadians(SampleSupplyTurnDeg))
          .build();

  @Override
  public void runOpMode() throws InterruptedException {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);

    drive = new SampleMecanumDrive(hardwareMap);

    // Score the first chamber
    // Push all three samples to the observation zone
    // Repeatedly score the high chamber with slightly different
    slide.stow();
    slide.backwardSlideExtension();
    liftClaw.closeClaw();
    liftClaw.foldLiftArm();
    //    drive.setPoseEstimate(trajs1.start());

    waitForStart();

    // spotless:off

    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                slide.aimCommand().beforeStarting(liftClaw::closeClaw).alongWith(
                        followTrajectory(start2Chamber1).alongWith(upLiftToChamber())
                ),
                hangAndStowLift(),

                followTrajectory(Chamber12Sample1).deadlineWith(
                        new WaitCommand(swipeDelay).andThen(slide.swipeCommand())
                ),
                slide.aimCommand().alongWith(
                        followTrajectory(Sample12Sample2)
                ).alongWith(new WaitCommand(swipeDelay).andThen(slide.swipeCommand())),

                followTrajectory(Chamber12Grab).alongWith(slide.aimCommand().andThen(
                        new InstantCommand(() -> slide.forwardSlideExtension()))),

                slide.grabCommand(),
                handoffAndLiftToChamber().alongWith(
                        new WaitCommand(handOff2TrajDelay).andThen(followTrajectory(Grab2Chamber2))
                ),
//                handoffAndLiftToChamber(lift, liftClaw, slide)
//                    .alongWith(new WaitCommand(2000).andThen(followTrajectory(drive, trajs3))),
                hangAndStowLift(),
                followTrajectory(Chamber22Grab).alongWith(slide.aimCommand().andThen(
                        new InstantCommand(() -> slide.forwardSlideExtension()))),

                slide.grabCommand(),
                handoffAndLiftToChamber().alongWith(
                        new WaitCommand(handOff2TrajDelay).andThen(followTrajectory(Grab2Chamber3))
                ),
                hangAndStowLift(),
                followTrajectory(Chamber32Grab).alongWith(slide.aimCommand().andThen(
                        new InstantCommand(() -> slide.forwardSlideExtension()))),

                slide.grabCommand(),
                handoffAndLiftToChamber().alongWith(
                        new WaitCommand(handOff2TrajDelay).andThen(followTrajectory(Grab2Chamber4))
                ),
                hangAndStowLift(),
                autoFinish()
            ));

    //spotless:on

    while (opModeIsActive() && !isStopRequested()) {
      lift.periodicTest();
      CommandScheduler.getInstance().run();
    }
  }
}
