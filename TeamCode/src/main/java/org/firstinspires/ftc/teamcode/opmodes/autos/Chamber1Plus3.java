package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

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
  AlphaLiftClaw liftClaw;
  Lift lift;
  AlphaSlide slide;

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
  //          TrajectoryManager.trajectorySequenceBuilder(new Pose2d(47.97, -66.91,
  // Math.toRadians(90.00)))
  //          .splineTo(new Vector2d(4.30, -34.47), Math.toRadians(90.00))
  //          .build();
  //
  //  TrajectorySequence chamberToFirst =
  //          TrajectoryManager.trajectorySequenceBuilder(new Pose2d(47.97, -66.91,
  // Math.toRadians(90.00)))
  //          .splineTo(new Vector2d(11.31, -43.76), Math.toRadians(17.84))
  //          .splineToConstantHeading(new Vector2d(34.99, -24.82), Math.toRadians(90.00))
  //          .splineToLinearHeading(new Pose2d(46.39, -14.65, Math.toRadians(0.00)),
  // Math.toRadians(0.00))
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
  //  public static double grabX = 36.60;
  //  public static double grabY = -60.31;
  public static Pose2dHelperClass grab = new Pose2dHelperClass(36.60, -60.31, 90.00);

  //  public static double chamberX = 6.49;
  //  public static double chamberY = -30.74;
  public static double gap = 2;
  public static Pose2dHelperClass chamber = new Pose2dHelperClass(6.49, -30, 90.00);
  public static Pose2dHelperClass chamber1 = new Pose2dHelperClass(chamber.X - gap, chamber.Y, 90.00);
  public static Pose2dHelperClass chamber2 =
      new Pose2dHelperClass(chamber.X - gap * 2, chamber.Y, 90.00);
  public static Pose2dHelperClass chamber3 =
      new Pose2dHelperClass(chamber.X - gap * 3, chamber.Y, 90.00);

  //  public static double startX = 24.43;
  //  public static double startY = -64.95;
  public static Pose2dHelperClass start = new Pose2dHelperClass(24.43, -64.95, 90.00);

  public static double yBottom = -50;

  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //    TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(24.43, -64.95,
    // Math.toRadians(90.00)))
    //            .lineTo(new Vector2d(5.8, -30.74))
    //            .build(); // 1+0

    //    TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory0.end())
    //            .lineToConstantHeading(new Vector2d(25.71, -39.80))
    ////            .splineToConstantHeading(new Vector2d(33, -28), Math.toRadians(75.96))
    //            .splineToSplineHeading(new Pose2d(47.49, -8.73, Math.toRadians(90.00)),
    // Math.toRadians(-90.00))
    //            .lineToSplineHeading(new Pose2d(47.33, yBottom, Math.toRadians(90.00)))
    //            .splineToSplineHeading(new Pose2d(53.58, -8.73, Math.toRadians(90.00)),
    // Math.toRadians(77.37))
    //            .lineToSplineHeading(new Pose2d(54.54, yBottom, Math.toRadians(90.00)))
    //            .build(); //

    TrajectorySequence push2Blocks = drive.trajectorySequenceBuilder(new Pose2d(1.37, -49.37, Math.toRadians(90.00)))
            .splineToConstantHeading(new Vector2d(35.66, -31.54), Math.toRadians(90.00))
            .splineToConstantHeading(new Vector2d(48.69, -12.11), Math.toRadians(-90.00))
            .lineToLinearHeading(new Pose2d(48.46, -53, Math.toRadians(90.00)), getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
            .splineToConstantHeading(new Vector2d(59.20, -18.74), Math.toRadians(-45.00))
            .lineToLinearHeading(new Pose2d(59.08, -53, Math.toRadians(90.00)), getVelocityConstraint(50, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
//            .splineToSplineHeading(new Pose2d(64.15, -14.54, Math.toRadians(180.00)), Math.toRadians(0.00))
//            .lineToLinearHeading(new Pose2d(64.15, -56.31, Math.toRadians(180.00)))
            .build(); // push 2 blocks

    TrajectorySequence push2Blocks1 =
        drive
            .trajectorySequenceBuilder(chamber.toPose2d())
            .lineToConstantHeading(new Vector2d(15.94, -49.58))
            .splineToConstantHeading(new Vector2d(36.76, -25.71), Math.toRadians(90.00))
            .splineToConstantHeading(new Vector2d(44.29, -13.54), Math.toRadians(0.00))
            .lineToConstantHeading(new Vector2d(44.61, -57.10))
            .splineToConstantHeading(new Vector2d(55.66, -13.05), Math.toRadians(0.00))
            .splineToConstantHeading(new Vector2d(64.79, -13.37), Math.toRadians(0.00))
            .build(); // push 2 blocks

    TrajectorySequence pushToGrab =
        drive
            .trajectorySequenceBuilder(push2Blocks.end())
            .lineToSplineHeading(new Pose2d(36.60, -60.31, Math.toRadians(90.00)))
            .build(); // push end to grab

//    TrajectorySequence grabToChamber =
//        drive
//            .trajectorySequenceBuilder(grab.toPose2d())
//            .lineToSplineHeading(new Pose2d(6.49, -30.74, Math.toRadians(90.00)))
//            .build(); // grab to chamber

    TrajectorySequence grabToChamber1 =
            drive
                    .trajectorySequenceBuilder(grab.toPose2d())
                    .lineToSplineHeading(chamber1.toPose2d())
                    .build(); // grab to chamber1
    TrajectorySequence grabToChamber2 =
            drive
                    .trajectorySequenceBuilder(grab.toPose2d())
                    .lineToSplineHeading(chamber2.toPose2d())
                    .build(); // grab to chamber2
    TrajectorySequence grabToChamber3 =
            drive
                    .trajectorySequenceBuilder(grab.toPose2d())
                    .lineToSplineHeading(chamber3.toPose2d())
                    .build(); // grab to chamber3


    TrajectorySequence chamberToGrab =
        drive
            .trajectorySequenceBuilder(chamber.toPose2d())
            .lineToConstantHeading(new Vector2d(36.60, -60.31))
            .build(); // chamber to grab

    TrajectorySequence startToChamber =
        drive
            .trajectorySequenceBuilder(start.toPose2d())
            .lineToConstantHeading(chamber.toVector2d())
            .build(); // start to chamber

    // Score the first chamber
    // Push all three samples to the observation zone
    // Repeatedly score the high chamber with slightly different
    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                initialize(liftClaw, slide),
                new InstantCommand(() -> drive.setPoseEstimate(startToChamber.start())),
                new AutoDriveCommand(drive, startToChamber)
                    .alongWith(grabToPreHang(lift, liftClaw))
                    .andThen(upToChamber(lift)),
//                    new WaitCommand(500),
                    stowArmFromBasket(lift, liftClaw),

                new AutoDriveCommand(drive, push2Blocks),

                new AutoDriveCommand(drive, pushToGrab),
                    chamberToGrab(lift, liftClaw),
                    liftClaw.closeClawCommand(),

                new AutoDriveCommand(drive, grabToChamber1)
                    .andThen(grabToPreHang(lift, liftClaw))
                    .andThen(upToChamber(lift)),

                new AutoDriveCommand(drive, chamberToGrab)
                        .alongWith(chamberToGrab(lift, liftClaw))
                        .andThen(liftClaw.closeClawCommand()),
                new AutoDriveCommand(drive, grabToChamber2)
                        .andThen(grabToPreHang(lift, liftClaw))
                        .andThen(upToChamber(lift)),

                new AutoDriveCommand(drive, chamberToGrab)
                        .alongWith(chamberToGrab(lift, liftClaw))
                        .andThen(liftClaw.closeClawCommand()),
                new AutoDriveCommand(drive, grabToChamber3)
                        .andThen(grabToPreHang(lift, liftClaw))
                        .andThen(upToChamber(lift)),

                    new AutoDriveCommand(drive, chamberToGrab)

                //                                    .andThen(new
                // InstantCommand(liftClaw::closeClaw))
                //                            ,
                //                            new AutoDriveCommand(drive, trajectory3)
                //                                    .alongWith(grabToPreHang(lift, liftClaw))
                //                                    .andThen(new WaitCommand(300))
                //                                    .andThen(upToChamber(lift))
                //                                    .andThen(new WaitCommand(500))
                //                                    .andThen(chamberToGrab(lift, liftClaw))

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
    //                        new AutoDriveCommand(drive, chamberToGrab).andThen(grabToPreHang(lift,
    // liftClaw)),
    //                        new AutoDriveCommand(drive, grabToChamber)
    //                            .andThen(upToChamber(lift))
    //                            .andThen(chamberToGrab(lift, liftClaw)),
    //                        new AutoDriveCommand(drive, chamberToGrab).andThen(grabToPreHang(lift,
    // liftClaw)),
    //                        new AutoDriveCommand(drive, grabToChamber)
    //                            .andThen(upToChamber(lift))
    //                            .andThen(chamberToGrab(lift, liftClaw)),
    //                        new AutoDriveCommand(drive, chamberToAscent)
    //                            .alongWith(autoFinish(liftClaw, lift, slide))
    //            ));

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
      lift.periodicTest();
    }
  }
}
