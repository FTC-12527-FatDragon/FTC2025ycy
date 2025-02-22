package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
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
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;

@Config
@Autonomous(name = "Basket 1+4", group = "Autos")
public class Basket1Plus4 extends AutoCommandBase {

    public static Pose2dHelperClass
            start = new Pose2dHelperClass(-31.64, -65.06, 0.00);
    public static Pose2dHelperClass basket =
            currentRobot == DriveConstants.RobotType.ALPHA ?
                    new Pose2dHelperClass(-56.76, -57.25, 45.00) :
                    new Pose2dHelperClass(-58.76, -59.25, 45.00);
    public static Pose2dHelperClass grab1 =
            currentRobot == DriveConstants.RobotType.ALPHA ?
                    new Pose2dHelperClass(-49.25, -52.43, 90.00) :
                    new Pose2dHelperClass(-49.25, -53.43, 90.00);
    public static Pose2dHelperClass grab2 = new Pose2dHelperClass(-60.03, -54.43, 90.00);
    public static Pose2dHelperClass grab3 = new Pose2dHelperClass(-42, -25.41, 180.00);
    public static Pose2dHelperClass climb = new Pose2dHelperClass(-22.78, -11.10, 180.00);
    public static Pose2dHelperClass grab4 = new Pose2dHelperClass(23.45, -58.82, 0.00);

    public static long waitDropTimeout = 200;
    public static long liftBackTimeout = 200;

    TrajectorySequence startToBasket;

    TrajectorySequence basketToGrab1;

    TrajectorySequence grab1ToBasket;

    TrajectorySequence basketToGrab2;

    TrajectorySequence grab2ToBasket;

    TrajectorySequence basketToGrab3;

    TrajectorySequence grab3ToBasket;

    TrajectorySequence basketToClimb;

    TrajectorySequence basketToGrab4;

    TrajectorySequence grab4ToBasket;


    public Command basket0() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(drive, startToBasket).alongWith(slide.aimCommand(), liftToBasket()),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command basket1() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(drive, basketToGrab1).alongWith(new WaitCommand(liftBackTimeout).andThen(liftBack())),
                grabAndBack(),
                new AutoDriveCommand(drive, grab1ToBasket).alongWith(slide.aimCommand(), liftToBasket()),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command basket2() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(drive, basketToGrab2).alongWith(new WaitCommand(liftBackTimeout).andThen(liftBack())),
                grabAndBack(),
                new AutoDriveCommand(drive, grab2ToBasket).alongWith(slide.aimCommand(), liftToBasket())
        );
    }

    public Command basket3() {
        return new SequentialCommandGroup(
                new WaitCommand(waitDropTimeout),
                new AutoDriveCommand(drive, basketToGrab3).alongWith(new WaitCommand(liftBackTimeout).andThen(liftBack())),
                grabAndBack3(),
                new AutoDriveCommand(drive, grab3ToBasket).alongWith(slide.aimCommand(), liftToBasket()),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command basket4() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(drive, basketToGrab4)
                        .alongWith(new WaitCommand(liftBackTimeout)
                                        .andThen(liftBack()),
                        forwardslideCommand()),
                grabWithoutForwardAndBack(),
                new AutoDriveCommand(drive, grab4ToBasket).alongWith(liftToBasket((long)grab4ToBasket.duration() * 1000)),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command climb() {
        return new AutoDriveCommand(drive, basketToClimb)
                .alongWith(new WaitCommand(liftBackTimeout)
                        .andThen(liftBack()).andThen(endClimbCommand()));
    }


    @Override
    public Command runAutoCommand() {
        startToBasket = drive.trajectorySequenceBuilder(start.toPose2d())
                .lineToSplineHeading(basket.toPose2d())
                .build();

        basketToGrab1 = drive.trajectorySequenceBuilder(basket.toPose2d())
                .lineToSplineHeading(grab1.toPose2d())
                .build();

        grab1ToBasket = drive.trajectorySequenceBuilder(grab1.toPose2d())
                .lineToSplineHeading(basket.toPose2d())
                .build();

        basketToGrab2 = drive.trajectorySequenceBuilder(basket.toPose2d())
                .lineToSplineHeading(grab2.toPose2d())
                .build();

        grab2ToBasket = drive.trajectorySequenceBuilder(grab2.toPose2d())
                .lineToSplineHeading(basket.toPose2d())
                .build();

        basketToGrab3 = drive.trajectorySequenceBuilder(basket.toPose2d())
                .lineToSplineHeading(grab3.toPose2d())
                .build();

        grab3ToBasket = drive.trajectorySequenceBuilder(grab3.toPose2d())
                .lineToSplineHeading(basket.toPose2d())
                .build();

        basketToClimb = drive.trajectorySequenceBuilder(basket.toPose2d())
                .splineToSplineHeading(climb.toPose2d(), Math.toRadians(0.00))
                .build();

        basketToGrab4 = drive.trajectorySequenceBuilder(basket.toPose2d())
                .lineToSplineHeading(grab4.toPose2d())
                .build();

        grab4ToBasket = drive.trajectorySequenceBuilder(grab4.toPose2d())
                .lineToSplineHeading(basket.toPose2d())
                .build();
        return new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPoseEstimate(startToBasket.start())),
                basket0(),
                basket1(),
                basket2(),
                basket3(),
                basket4(),
                climb()
        );
    }
}
