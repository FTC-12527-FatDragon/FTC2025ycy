package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
@Autonomous(name = "Chamber 1+3", group = "Autos")
public class Chamber1Plus3 extends LinearOpMode {
  // For Chamber Scoring
  public static double xChamber = 10;
  public static double yChamber = 20;
  public static double headingChamber = -45;

  // The rightmost sample
  public static double xRightMost = 25;
  public static double yRightMost = 9.5;
  public static double headingRightMost = 0;

  // The middle sample
  public static double xMiddle = 0;
  public static double yMiddle = 0;
  public static double headingMiddle = 0;

  // The leftmost sample
  public static double xLeftMost = 0;
  public static double yLeftMost = 0;
  public static double headingLeftMost = 0;

  // The Observation zone
  public static double xObservation = 0;
  public static double yObservation = 0;
  public static double headingObservation = 0;

  // The Grab position
  public static double xGrab = 0;
  public static double yGrab = 0;
  public static double headingGrab = 0;

  // The Ascent zone
  public static double xAscent = 0;
  public static double yAscent = 0;
  public static double headingAscent = 0;

  AlphaLiftClaw liftClaw;
  AlphaLift lift;
  AlphaSlide slide;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

  // Start to Basket
  TrajectorySequence firstBlock =
      TrajectoryManager.trajectorySequenceBuilder(startPose)
          .lineToLinearHeading(new Pose2d(xRightMost, yRightMost, Math.toRadians(headingRightMost)))
          .build();

  // Basket to the rightmost sample
  TrajectorySequence firstToObservation =
      TrajectoryManager.trajectorySequenceBuilder(firstBlock.end())
          .lineToLinearHeading(new Pose2d(xObservation, yObservation, Math.toRadians(headingObservation)))
          .build();

  // rightmost sample to basket
  TrajectorySequence observationToSecond =
      TrajectoryManager.trajectorySequenceBuilder(firstToObservation.end())
          .lineToLinearHeading(new Pose2d(xMiddle, yMiddle, Math.toRadians(headingMiddle)))
          .build();

  // basket to middle sample
  TrajectorySequence secondToObservation =
      TrajectoryManager.trajectorySequenceBuilder(observationToSecond.end())
          .lineToLinearHeading(new Pose2d(xObservation, yObservation, Math.toRadians(headingObservation)))
          .build();

  // middle sample to basket
  TrajectorySequence observationToThird =
      TrajectoryManager.trajectorySequenceBuilder(secondToObservation.end())
          .lineToLinearHeading(new Pose2d(xLeftMost, yLeftMost, Math.toRadians(headingLeftMost)))
          .build();

  // basket to leftmost sample
  TrajectorySequence thirdToObservation =
      TrajectoryManager.trajectorySequenceBuilder(observationToThird.end())
          .lineToLinearHeading(new Pose2d(xObservation, yObservation, Math.toRadians(headingObservation)))
          .build();

  // leftmost sample to basket
  TrajectorySequence observationToGrab =
      TrajectoryManager.trajectorySequenceBuilder(thirdToObservation.end())
          .lineToLinearHeading(new Pose2d(xChamber, yChamber, Math.toRadians(headingChamber)))
          .build();

  // basket to ascent zone
  TrajectorySequence grabToChamber =
      TrajectoryManager.trajectorySequenceBuilder(observationToGrab.end())
          .lineToLinearHeading(new Pose2d(xChamber, yChamber, Math.toRadians(headingChamber)))
          .build();

  // basket to ascent zone
  TrajectorySequence chamberToGrab =
          TrajectoryManager.trajectorySequenceBuilder(grabToChamber.end())
                  .lineToLinearHeading(new Pose2d(xGrab, yGrab, Math.toRadians(headingGrab)))
                  .build();

  TrajectorySequence chamberToAscent =
          TrajectoryManager.trajectorySequenceBuilder(grabToChamber.end())
                  .lineToLinearHeading(new Pose2d(xAscent, yAscent, Math.toRadians(headingAscent)))
                  .build();

  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new AlphaLift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap,telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    // Score the first chamber
    // Push all three samples to the observation zone
    // Repeatedly score the high chamber with slightly different
    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                initialize(liftClaw, slide),

                new AutoDriveCommand(drive, grabToChamber).alongWith(grabToPreHang(lift, liftClaw)),
                chamberToGrab(lift, liftClaw),

                new AutoDriveCommand(drive, firstBlock),
                new AutoDriveCommand(drive, firstToObservation),
                new AutoDriveCommand(drive, observationToSecond),
                new AutoDriveCommand(drive, secondToObservation),
                new AutoDriveCommand(drive, observationToThird),
                new AutoDriveCommand(drive, thirdToObservation),

                new AutoDriveCommand(drive, observationToGrab)
                        .andThen(grabToPreHang(lift, liftClaw)),
                new AutoDriveCommand(drive, grabToChamber).andThen(upToChamber(lift))
                        .andThen(chamberToGrab(lift, liftClaw)),

                new AutoDriveCommand(drive, chamberToGrab)
                        .andThen(grabToPreHang(lift, liftClaw)),
                new AutoDriveCommand(drive, grabToChamber).andThen(upToChamber(lift))
                        .andThen(chamberToGrab(lift, liftClaw)),

                new AutoDriveCommand(drive, chamberToGrab)
                        .andThen(grabToPreHang(lift, liftClaw)),
                new AutoDriveCommand(drive, grabToChamber).andThen(upToChamber(lift))
                        .andThen(chamberToGrab(lift, liftClaw)),

                new AutoDriveCommand(drive, chamberToAscent).alongWith(autoFinish(liftClaw, lift, slide))

            )
        );


    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
    }
  }
}
