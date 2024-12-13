package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config
@Autonomous(name = "Test Auto", group = "Autos")
public class AutoTest extends LinearOpMode {

  LiftClaw liftClaw;
  Lift lift;
  SlideSuperStucture slide;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

  /*
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
  */

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
                        new SequentialCommandGroup(
                                slide.aimCommand().beforeStarting(liftClaw::closeClaw),
                                upLiftToBasket(lift, liftClaw)
                        ));

        // spotless:on

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
    }
  }
}
