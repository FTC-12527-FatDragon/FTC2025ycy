package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
//  // For Chamber Scoring
//  public static double xChamber = 19;
//  public static double yChamber = 0;
//  public static double headingChamber = -180;
//
//  // The transfer point
//  public static double xTransfer = 24;
//  public static double yTransfer = 18;
//  public static double headingTransfer = 0;
//
//  // The leftmost sample
//  public static double xRightMost = 30;
//  public static double yRightMost = 20;
//  public static double headingRightMost = -90;
//
//  // The middle sample
//  public static double xMiddle = 26;
//  public static double yMiddle = 30;
//  public static double headingMiddle = -90;
//
//  // The rightmost sample
//  public static double xLeftMost = 30;
//  public static double yLeftMost = 31;
//  public static double headingLeftMost = -90;
//
//  // The Observation zone
//  public static double xObservation = 12;
//  public static double yObservation = 30;
//  public static double headingObservation = -90;
//
//  // The Grab position
//  public static double xGrab = 5;
//  public static double yGrab = 24;
//  public static double headingGrab = -180;
//
//  // The Ascent zone
//  public static double xAscent = 36;
//  public static double yAscent = 12;
//  public static double headingAscent = 0;
//
//  AlphaLiftClaw liftClaw;
//  AlphaLift lift;
//  AlphaSlide slide;
//
//
//
//
//
//  Pose2d chamberPose = new Pose2d(xChamber, yChamber, Math.toRadians(headingChamber));
//  Pose2d leftPose = new Pose2d(xLeftMost, yLeftMost, Math.toRadians(headingLeftMost));
//  Pose2d middlePose = new Pose2d(xMiddle, yMiddle, Math.toRadians(headingMiddle));
//  Pose2d rightPose = new Pose2d(xRightMost, yRightMost, Math.toRadians(headingRightMost));
//  Pose2d grabPose = new Pose2d(xGrab, yGrab, Math.toRadians(headingGrab));
//  Pose2d transferPose = new Pose2d(xTransfer, yTransfer, headingTransfer);
//  Pose2d ascentPose = new Pose2d(xAscent, yAscent, Math.toRadians(headingAscent));
//  Pose2d observationPose =
//          new Pose2d(xObservation, yObservation, Math.toRadians(headingObservation));
//
//  TrajectorySequence startToChamber =
//          TrajectoryManager.trajectorySequenceBuilder(new Pose2d(47.97, -66.91, Math.toRadians(90.00)))
//          .splineTo(new Vector2d(4.30, -34.47), Math.toRadians(90.00))
//          .build();
//
//  TrajectorySequence chamberToFirst =
//          TrajectoryManager.trajectorySequenceBuilder(new Pose2d(47.97, -66.91, Math.toRadians(90.00)))
//          .splineTo(new Vector2d(11.31, -43.76), Math.toRadians(17.84))
//          .splineToConstantHeading(new Vector2d(34.99, -24.82), Math.toRadians(90.00))
//          .splineToLinearHeading(new Pose2d(46.39, -14.65, Math.toRadians(0.00)), Math.toRadians(0.00))
//          .build();
//
//  TrajectorySequence firstToObservation =
//          TrajectoryManager.trajectorySequenceBuilder(chamberToFirst.end())
//              .lineToConstantHeading(new Vector2d(46.22, -66.04))
//              .build();
//
//  // rightmost sample to basket
//  TrajectorySequence observationToSecond =
//      TrajectoryManager.trajectorySequenceBuilder(firstToObservation.end())
//              .splineToConstantHeading(new Vector2d(55.34, -13.94), Math.toRadians(0.00))
//          .build();
//
//  TrajectorySequence secondToObservation =
//      TrajectoryManager.trajectorySequenceBuilder(observationToSecond.end())
//              .lineToConstantHeading(new Vector2d(54.99, -66.91))
//          .build();
//
//  // middle sample to basket
//  TrajectorySequence observationToThird =
//      TrajectoryManager.trajectorySequenceBuilder(secondToObservation.end())
//              .splineToConstantHeading(new Vector2d(62.70, -13.94), Math.toRadians(0.00))
//          .build();
//
//  // basket to rightmost sample
//  TrajectorySequence thirdToObservation =
//      TrajectoryManager.trajectorySequenceBuilder(observationToThird.end())
//              .lineToConstantHeading(new Vector2d(62.70, -65.86))
//          .build();
//
//  // leftmost sample to basket
//  TrajectorySequence observationToGrab =
//      TrajectoryManager.trajectorySequenceBuilder(thirdToObservation.end())
//          .lineToLinearHeading(new Pose2d(42.09, -47.65, Math.toRadians(-90.00)))
//          .splineToConstantHeading(new Vector2d(34.78, -62.96), Math.toRadians(-90.00))
//          .build();
//
//  // basket to ascent zone
//  TrajectorySequence grabToChamber =
//          TrajectoryManager.trajectorySequenceBuilder(observationToGrab.end())
//          .splineTo(new Vector2d(4.30, -34.47), Math.toRadians(90.00))
//          .build();
//
//  // basket to ascent zone
//  TrajectorySequence chamberToGrab =
//      TrajectoryManager.trajectorySequenceBuilder(chamberPose)
//          .splineToLinearHeading(grabPose, Math.toRadians(-180))
//          .build();
//
//  TrajectorySequence chamberToAscent =
//      TrajectoryManager.trajectorySequenceBuilder(chamberPose)
//          .splineToLinearHeading(transferPose, Math.toRadians(0))
//          .splineToLinearHeading(ascentPose, Math.toRadians(90))
//          .build();


  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new AlphaLift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);


    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



    TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(24.43, -64.95, Math.toRadians(90.00)))
            .lineTo(new Vector2d(6.49, -30.74))
            .build(); // 1+0


    TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(6.49, -30.74, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(35.32, -43.81, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(35.16, -22.83, Math.toRadians(71.10)))
            .lineToSplineHeading(new Pose2d(47.33, -9.21, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(47.97, -55.82, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(50.86, -14.18, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(59.51, -10.65, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(59.67, -55.98, Math.toRadians(90.00)))
            .build();
    // push 2 blocks

    TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(59.67, -55.98, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(36.60, -60.31, Math.toRadians(90.00)))
            .lineToSplineHeading(new Pose2d(6.49, -30.74, Math.toRadians(90.00)))
            .build();
    // push end to grab to chamber












    drive.setPoseEstimate(trajectory0.start());

    // Score the first chamber
    // Push all three samples to the observation zone
    // Repeatedly score the high chamber with slightly different
    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                    initialize(liftClaw, slide),
            new AutoDriveCommand(drive, trajectory0)
                .alongWith(grabToPreHang(lift, liftClaw))
                        .andThen(new WaitCommand(300))
                        .andThen(upToChamber(lift))
                        .andThen(new WaitCommand(500))
                        .andThen(chamberToGrab(lift, liftClaw)),
            new AutoDriveCommand(drive, trajectory1),
            new AutoDriveCommand(drive, trajectory2),
                      new AutoDriveCommand(drive, trajectory3)
                              .alongWith(grabToPreHang(lift, liftClaw))
                              .andThen(new WaitCommand(300))
                              .andThen(upToChamber(lift))
                              .andThen(new WaitCommand(500))
                              .andThen(chamberToGrab(lift, liftClaw))


//                ,
//                        new AutoDriveCommand(drive, chamberToFirst),
//                        new AutoDriveCommand(drive, firstToObservation),
        ));
//                        new AutoDriveCommand(drive, observationToSecond),
//                        new AutoDriveCommand(drive, secondToObservation),
//                        new AutoDriveCommand(drive, observationToThird),
//                        new AutoDriveCommand(drive, thirdToObservation),
//                        new AutoDriveCommand(drive, observationToGrab)
//                            .andThen(grabToPreHang(lift, liftClaw)),
//                        new AutoDriveCommand(drive, grabToChamber)
//                            .andThen(upToChamber(lift))
//                            .andThen(chamberToGrab(lift, liftClaw)),
//                        new AutoDriveCommand(drive, chamberToGrab).andThen(grabToPreHang(lift, liftClaw)),
//                        new AutoDriveCommand(drive, grabToChamber)
//                            .andThen(upToChamber(lift))
//                            .andThen(chamberToGrab(lift, liftClaw)),
//                        new AutoDriveCommand(drive, chamberToGrab).andThen(grabToPreHang(lift, liftClaw)),
//                        new AutoDriveCommand(drive, grabToChamber)
//                            .andThen(upToChamber(lift))
//                            .andThen(chamberToGrab(lift, liftClaw)),
//                        new AutoDriveCommand(drive, chamberToAscent)
//                            .alongWith(autoFinish(liftClaw, lift, slide))
//            ));

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
    }
  }
}
