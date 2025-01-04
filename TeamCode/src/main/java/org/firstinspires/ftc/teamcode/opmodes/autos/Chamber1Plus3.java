package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
  AlphaLiftClaw liftClaw;
  Lift lift;
  AlphaSlide slide;

  public static Pose2dHelperClass grab = new Pose2dHelperClass(36, -58, 90.00);

  public static double gap = 2;
  public static Pose2dHelperClass chamber = new Pose2dHelperClass(5.49, -29.5, 90.00);
  public static Pose2dHelperClass chamber1 = new Pose2dHelperClass(chamber.X - gap, chamber.Y, 90.00);
  public static Pose2dHelperClass chamber2 =
          new Pose2dHelperClass(chamber.X - gap * 2, chamber.Y, 90.00);
  public static Pose2dHelperClass chamber3 =
          new Pose2dHelperClass(chamber.X - gap * 3, chamber.Y, 90.00);
  public static Pose2dHelperClass chamber4 =
          new Pose2dHelperClass(chamber.X - gap * 4, chamber.Y, 90.00);

  //  public static double startX = 24.43;
  //  public static double startY = -64.95;
  public static Pose2dHelperClass start = new Pose2dHelperClass(24.43, -64.95, 90.00);

  public static long Grab2ChamberUpperDelay = 0;

  private SampleMecanumDrive drive;

  public Command obersvationToChamberCycle(TrajectorySequence toChamberSequence, TrajectorySequence chamberToGrab){
    return new SequentialCommandGroup(
            liftClaw.closeClawCommand(),

            new AutoDriveCommand(drive, toChamberSequence)
                    .alongWith(new WaitCommand(Grab2ChamberUpperDelay).andThen(toPreHang(lift, liftClaw))),

            upToChamber(lift),

            new AutoDriveCommand(drive, chamberToGrab)
                    .alongWith(chamberToGrab(lift, liftClaw))
    );
  }

  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    Telemetry telemetry_M = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry_M);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry_M);
    slide = new AlphaSlide(hardwareMap, telemetry_M);

    drive = new SampleMecanumDrive(hardwareMap);

    TrajectorySequence push2Blocks = drive.trajectorySequenceBuilder(new Pose2d(1.37, -49.37, Math.toRadians(90.00)))
            .lineToConstantHeading(new Vector2d(18.69, -31.62))
            .splineToConstantHeading(new Vector2d(34.62, -27.69), Math.toRadians(90.00))
            .splineToConstantHeading(new Vector2d(44.77, -14.54), Math.toRadians(-70.00))
            .lineToLinearHeading(new Pose2d(48.46, -53, Math.toRadians(90.00)), getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
            .splineToConstantHeading(new Vector2d(57.69, -15.46), Math.toRadians(-45.00))
            .lineToLinearHeading(new Pose2d(59.08, -53, Math.toRadians(90.00)), getVelocityConstraint(50, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
//            .splineToSplineHeading(new Pose2d(64.15, -14.54, Math.toRadians(180.00)), Math.toRadians(0.00))
//            .lineToLinearHeading(new Pose2d(64.15, -56.31, Math.toRadians(180.00)))
            .build(); // push 2 blocks

    TrajectorySequence pushToGrab =
            drive
                    .trajectorySequenceBuilder(push2Blocks.end())
                    .lineToSplineHeading(new Pose2d(36.60, -60.31, Math.toRadians(90.00)))
                    .build(); // push end to grab

    TrajectorySequence chamberToGrab = drive.trajectorySequenceBuilder(chamber3.toPose2d())
            .lineToConstantHeading(chamber3.toVector2d().plus(new Vector2d(0, -1)))
            .splineToConstantHeading(grab.toVector2d(), Math.toRadians(-90))
            .build();





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

    TrajectorySequence grabToChamber4 =
            drive
                    .trajectorySequenceBuilder(grab.toPose2d())
                    .lineToSplineHeading(chamber4.toPose2d())
                    .build();

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
                            liftClaw.closeClawCommand(),
                            new AutoDriveCommand(drive, startToChamber).alongWith(new WaitCommand(Grab2ChamberUpperDelay).andThen(toPreHang(lift, liftClaw))),

                            upToChamber(lift),

//                    stowArmFromBasket(lift, liftClaw),

                            new AutoDriveCommand(drive, push2Blocks).alongWith(chamberToGrab(lift, liftClaw)),
//                    new WaitCommand(500).deadlineWith(lift.manualResetCommand()),
//
                            new AutoDriveCommand(drive, pushToGrab),
                            //.alongWith(new WaitCommand(500).deadlineWith(lift.manualResetCommand()))

                            obersvationToChamberCycle(grabToChamber1, chamberToGrab),

                            obersvationToChamberCycle(grabToChamber2, chamberToGrab),

                            obersvationToChamberCycle(grabToChamber3, chamberToGrab)

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

    waitForStart();

    int i=0;
    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
      lift.periodicTest();
      telemetry_M.addData("Iterative count", i);
      i++;
      telemetry_M.update();
    }
  }
}
