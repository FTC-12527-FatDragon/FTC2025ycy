package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

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
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
@Autonomous(name = "Chamber 1+3", group = "Autos")
public class Chamber1Plus3 extends LinearOpMode {
  // For Chamber Scoring
  public static double xChamber = 19;
  public static double yChamber = 0;
  public static double headingChamber = 0;

  // The transfer point
  public static double xTransfer = 24;
  public static double yTransfer = 18;
  public static double headingTransfer = 0;

  // The leftmost sample
  public static double xRightMost = 30;
  public static double yRightMost = 20;
  public static double headingRightMost = -90;

  // The middle sample
  public static double xMiddle = 30;
  public static double yMiddle = 26;
  public static double headingMiddle = -90;

  // The rightmost sample
  public static double xLeftMost = 30;
  public static double yLeftMost = 31;
  public static double headingLeftMost = -90;

  // The Observation zone
  public static double xObservation = 12;
  public static double yObservation = 30;
  public static double headingObservation = -90;

  // The Grab position
  public static double xGrab = 5;
  public static double yGrab = 24;
  public static double headingGrab = -180;

  // The Ascent zone
  public static double xAscent = 36;
  public static double yAscent = 12;
  public static double headingAscent = 0;

  AlphaLiftClaw liftClaw;
  AlphaLift lift;
  AlphaSlide slide;

  Pose2d startPose = new Pose2d(19, 0 , Math.toRadians(0));

  // Start to Basket
  TrajectorySequence chamberToFirst =
      TrajectoryManager.trajectorySequenceBuilder(startPose)
          .splineToLinearHeading(new Pose2d(xTransfer, yTransfer, Math.toRadians(headingTransfer)),
                  Math.toRadians(0))
          .splineToLinearHeading(new Pose2d(xLeftMost, yLeftMost, Math.toRadians(headingLeftMost)),
                  Math.toRadians(-90))
          .build();

  // Basket to the rightmost sample
  TrajectorySequence firstToObservation =
      TrajectoryManager.trajectorySequenceBuilder(chamberToFirst.end())
          .lineToLinearHeading(new Pose2d(xObservation, yObservation, Math.toRadians(headingObservation)))
          .build();

  // rightmost sample to basket
  TrajectorySequence observationToSecond =
      TrajectoryManager.trajectorySequenceBuilder(firstToObservation.end())
          .lineToLinearHeading(new Pose2d(xTransfer, yTransfer, Math.toRadians(headingTransfer)))
          .splineToLinearHeading(new Pose2d(xMiddle, yMiddle, Math.toRadians(headingMiddle)),
                  Math.toRadians(-180))
          .build();

  // basket to middle sample
  TrajectorySequence secondToObservation =
      TrajectoryManager.trajectorySequenceBuilder(observationToSecond.end())
          .lineToLinearHeading(new Pose2d(xObservation, yObservation, Math.toRadians(headingObservation)))
          .build();

  // middle sample to basket
  TrajectorySequence observationToThird =
      TrajectoryManager.trajectorySequenceBuilder(secondToObservation.end())
          .lineToLinearHeading(new Pose2d(xTransfer, yTransfer, Math.toRadians(headingTransfer)))
          .splineToLinearHeading(new Pose2d(xRightMost, yRightMost, Math.toRadians(headingRightMost)),
                   Math.toRadians(-180))
          .build();

  // basket to leftmost sample
  TrajectorySequence thirdToObservation =
      TrajectoryManager.trajectorySequenceBuilder(observationToThird.end())
          .lineToLinearHeading(new Pose2d(xObservation, yObservation, Math.toRadians(headingObservation)))
          .build();

  // leftmost sample to basket
  TrajectorySequence observationToGrab =
      TrajectoryManager.trajectorySequenceBuilder(thirdToObservation.end())
          .splineToLinearHeading(new Pose2d(xChamber, yChamber, Math.toRadians(headingChamber)),
                  Math.toRadians(-180))
          .build();

  // basket to ascent zone
  TrajectorySequence grabToChamber =
      TrajectoryManager.trajectorySequenceBuilder(observationToGrab.end())
          .splineToLinearHeading(new Pose2d(xChamber, yChamber, Math.toRadians(headingChamber)),
                  Math.toRadians(0))
          .forward(3)
          .strafeLeft(3)
          .build();

  // basket to ascent zone
  TrajectorySequence chamberToGrab =
          TrajectoryManager.trajectorySequenceBuilder(grabToChamber.end())
              .splineToLinearHeading(new Pose2d(xGrab, yGrab, Math.toRadians(headingGrab)),
                      Math.toRadians(-180))
              .build();

  TrajectorySequence chamberToAscent =
          TrajectoryManager.trajectorySequenceBuilder(grabToChamber.end())
              .splineToLinearHeading(new Pose2d(xTransfer, yTransfer, Math.toRadians(headingTransfer)),
                      Math.toRadians(0))
              .splineToLinearHeading(new Pose2d(xAscent, yAscent, Math.toRadians(headingAscent)),
                      Math.toRadians(90))
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

                new AutoDriveCommand(drive, chamberToFirst),
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
