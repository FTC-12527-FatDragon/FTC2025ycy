package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;
import org.firstinspires.ftc.teamcode.utils.Translation2dHelperClass;

@Config
@Autonomous(name = "Chamber 1+4", group = "Autos")
public class Chamber1Plus4 extends AutoCommandBase {

    public static Pose2dHelperClass grab = new Pose2dHelperClass(36, -60, 90.00);

    public static double gap = 3;
    public static Pose2dHelperClass chamber = new Pose2dHelperClass(5, -29, 90.00);
    public static Pose2dHelperClass chamber1 = new Pose2dHelperClass(chamber.X - gap, chamber.Y, 90.00);
    public static Pose2dHelperClass chamber2 =
            new Pose2dHelperClass(chamber.X - gap * 2, chamber.Y, 90.00);
    public static Pose2dHelperClass chamber3 =
            new Pose2dHelperClass(chamber.X - gap * 3, chamber.Y, 90.00);
    public static Pose2dHelperClass chamber4 =
            new Pose2dHelperClass(chamber.X - gap * 4, chamber.Y, 90.00);

    public static Translation2dHelperClass Sample1 = new Translation2dHelperClass(49.22, -26.44);
    public static Translation2dHelperClass Sample2 = new Translation2dHelperClass(59.77, -26.08);


    public static Pose2dHelperClass sample1Observation = new Pose2dHelperClass(48.46, -53, 90);

    //  public static double startX = 24.43;
    //  public static double startY = -64.95;
    public static Pose2dHelperClass start = new Pose2dHelperClass(7.67, -64.95, 90.00);

    public static long ChamberUp2ExtendSlideToSample1Delay = 1000;
    public static double GrabCycleReleaseOffsetSec = -0.5;

    public Command pushBlocksCycle(TrajectorySequence grab2DropSequence){
        return new SequentialCommandGroup(
//                new WaitCommand(1000),
                slide.grabCommand(),
//                new WaitCommand(3000),
                new AutoDriveCommand(drive, grab2DropSequence)
//                new InstantCommand(slide:),
//                slide.aimCommand().andThen(new AutoDriveCommand(drive, drop2Next).alongWith(drop2NextRun))
        );
    }

    @Override
    public Command runAutoCommand() {

        TrajectorySequence push2Blocks = drive.trajectorySequenceBuilder(new Pose2d(2.54, -32.08, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(6.46, -40.38))
                .splineToConstantHeading(new Vector2d(34.62, -27.69), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(47.54, -14.31), Math.toRadians(0.00))
                .lineToLinearHeading(sample1Observation.toPose2d(), getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
                .lineToLinearHeading(sample1Observation.toPose2d().plus(new Pose2d(0, 2, 0)))
                .splineToConstantHeading(new Vector2d(54.23, -16.62), Math.toRadians(0), getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
                .splineToConstantHeading(Sample2.toVector2d(), Math.toRadians(-90.00), getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
                .splineToLinearHeading(new Pose2d(59.08, -53, Math.toRadians(90.00)), Math.toRadians(-90), getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
//            .splineToSplineHeading(new Pose2d(64.15, -14.54, Math.toRadians(180.00)), Math.toRadians(0.00))
//            .lineToLinearHeading(new Pose2d(64.15, -56.31, Math.toRadians(180.00)))
                .build(); // push 2 blocks

        TrajectorySequence chamber2Sample1 = drive.trajectorySequenceBuilder(push2Blocks.start())
                .lineToSplineHeading(new Pose2d(6.46, -36.46, Math.toRadians(26.17)))
                .splineToLinearHeading(new Pose2d(25.85, -37, Math.toRadians(26.17)), Math.toRadians(27.51), getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(30))
                .build();

        TrajectorySequence grabSample12Observation2Sample2 = drive.trajectorySequenceBuilder(chamber2Sample1.end())
                .lineToLinearHeading(new Pose2d(30.23, -54, Math.toRadians(-20.71)))//, getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
                .UNSTABLE_addTemporalMarkerOffset(GrabCycleReleaseOffsetSec, () -> schedule(slide.aimCommand()))
                .lineToLinearHeading(new Pose2d(35.54, -38.31, Math.toRadians(29.81)))
                .build();

//        TrajectorySequence observation2Sample2 = drive.trajectorySequenceBuilder(new Pose2d(24.00, -57.46, Math.toRadians(-20.71)))
//                .lineToLinearHeading(new Pose2d(35.54, -38.31, Math.toRadians(29.81)))//, getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
//                .build();

        TrajectorySequence grabSample22Observation2Sample3 = drive.trajectorySequenceBuilder(grabSample12Observation2Sample2.end())
                .lineToLinearHeading(new Pose2d(39.91, -51.14, Math.toRadians(-30)))
                .UNSTABLE_addTemporalMarkerOffset(GrabCycleReleaseOffsetSec, () -> schedule(slide.aimCommand()))
                .lineToLinearHeading(new Pose2d(45.17, -37.76, Math.toRadians(25.89)))
                .build();

//        TrajectorySequence observation2Sample3 = drive.trajectorySequenceBuilder(grabSample22Observation.end())
//                .lineToLinearHeading(new Pose2d(45.17, -37.76, Math.toRadians(25.89)))
//                .build();

        TrajectorySequence grabSample32Observation2Grab = drive.trajectorySequenceBuilder(grabSample22Observation2Sample3.end())
                .lineToLinearHeading(new Pose2d(36.92, -52.15, Math.toRadians(-25.39)))
                .UNSTABLE_addTemporalMarkerOffset(GrabCycleReleaseOffsetSec, () -> schedule(slide.aimCommand().andThen(new InstantCommand(() -> slide.autoBackSlideExtension()))))
                .splineToLinearHeading(grab.toPose2d(), Math.toRadians(-90), getVelocityConstraint(25, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getACCEL_CONSTRAINT())
                .build();


//        TrajectorySequence observationToGrab =
//                drive
//                        .trajectorySequenceBuilder(grabSample32Observation2Grab.end())
//
//                        .build(); // 3 sample to grab

        Vector2d grabOffsetByChamber3 = grab.toVector2d().minus(chamber3.toVector2d());

        TrajectorySequence chamberToGrab = drive.trajectorySequenceBuilder(chamber3.toPose2d())
//                .lineToConstantHeading(chamber3.toVector2d().plus(grabOffsetByChamber3.times(0.9)))
//                .setVelConstraint(getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH))
//                .lineToConstantHeading(chamber3.toVector2d().plus(new Vector2d(0, -1)))
                .setAccelConstraint(getAccelerationConstraint(35))
                .lineToConstantHeading(grab.toVector2d())
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

        telemetry_M.addData("startToChamber.duration", startToChamber.duration());

        // Score the first chamber
        // Push all three samples to the observation zone
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drive.setPoseEstimate(startToChamber.start());
                    slide.autoBackSlideExtension();
                }),
                liftClaw.closeClawCommand(0),
                new AutoDriveCommand(drive, startToChamber).raceWith(new WaitCommand((long)(startToChamber.duration()*1000)))
                        .alongWith(new WaitCommand(Grab2PreHangDelay).andThen(toPreHang()))
                        .alongWith(new WaitCommand((long)(startToChamber.duration()*1000)+ChamberUpOffsetMs).andThen(upToChamber())),

                new AutoDriveCommand(drive, chamber2Sample1)
                        .alongWith(
                                chamberToGrab()
                        ).alongWith(
                                new WaitCommand(ChamberUp2ExtendSlideToSample1Delay).andThen(
                                        (new InstantCommand(slide::autoForwardSlideExtension).andThen(new WaitCommand(slide.slideRetractFar))).alongWith(slide.aimCommand(AlphaSlide.TurnServo.LEFT_55))
                                )
                        ),

                pushBlocksCycle(grabSample12Observation2Sample2),
                pushBlocksCycle(grabSample22Observation2Sample3),
                pushBlocksCycle(grabSample32Observation2Grab),
//                new AutoDriveCommand(drive, observationToGrab).alongWith(),
                //.alongWith(new WaitCommand(500).deadlineWith(lift.manualResetCommand()))

                observationToChamberCycle(grabToChamber1, chamberToGrab),
                observationToChamberCycle(grabToChamber2, chamberToGrab),
                observationToChamberCycle(grabToChamber3, chamberToGrab),
                observationToChamberCycle(grabToChamber4, chamberToGrab)
        );
    }
}
