package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.PPDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;

@Config
@Autonomous(name = "Basket 1+4 PP", group = "Autos")
public class Basket1Plus4PP extends AutoCommandBase {
    public static Pose
            start = new Pose(-31.64, -65.06, 0.00);
    public static Pose basket =
            currentRobot == DriveConstants.RobotType.ALPHA ?
                    new Pose(-56.76, -57.25, 45.00) :
                    new Pose(-58.76, -57.25, 45.00);
    public static Pose grab1 =
            currentRobot == DriveConstants.RobotType.ALPHA ?
                    new Pose(-49.25, -52.43, 90.00) :
                    new Pose(-49.25, -53.43, 90.00);
    public static Pose grab2 = new Pose(-60.03, -54.43, 90.00);
    public static Pose grab3 = new Pose(-49.88, -45.78, 135.00);
    public static Pose climb = new Pose(-22.78, -11.10, 180.00);
    public static Pose grab4 = new Pose(23.45, -58.82, 0.00);

    public static long waitDropTimeout = 200;
    public static long liftBackTimeout = 200;

    PathChain startToBasket, basketToGrab1, grab1ToBasket, basketToGrab2, grab2ToBasket;
    PathChain basketToGrab3, grab3ToBasket, basketToClimb, basketToGrab4, grab4ToBasket;


    public Command basket0() {
        return new SequentialCommandGroup(
                new PPDriveCommand(follower, startToBasket).alongWith(slide.aimCommand(), liftToBasket()),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command basket1() {
        return new SequentialCommandGroup(
                new PPDriveCommand(follower, basketToGrab1).alongWith(new WaitCommand(liftBackTimeout).andThen(liftBack())),
                grabAndBack(),
                new PPDriveCommand(follower, grab1ToBasket).alongWith(slide.aimCommand(), liftToBasket()),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command basket2() {
        return new SequentialCommandGroup(
                new PPDriveCommand(follower, basketToGrab2).alongWith(new WaitCommand(liftBackTimeout).andThen(liftBack())),
                grabAndBack(),
                new PPDriveCommand(follower, grab2ToBasket).alongWith(slide.aimCommand(), liftToBasket())
        );
    }

    public Command basket3() {
        return new SequentialCommandGroup(
                new WaitCommand(waitDropTimeout),
                new PPDriveCommand(follower, basketToGrab3).alongWith(new WaitCommand(liftBackTimeout).andThen(liftBack())),
                grabAndBack3(),
                new PPDriveCommand(follower, grab3ToBasket).alongWith(slide.aimCommand(), liftToBasket()),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command basket4() {
        return new SequentialCommandGroup(
                new PPDriveCommand(follower, basketToGrab4)
                        .alongWith(new WaitCommand(liftBackTimeout)
                                        .andThen(liftBack()),
                                forwardslideCommand()),
                grabWithoutForwardAndBack(),
                new PPDriveCommand(follower, grab4ToBasket).alongWith(liftToBasket(1000)),
                new WaitCommand(waitDropTimeout)
        );
    }

    public Command climb() {
        return new PPDriveCommand(follower, basketToClimb)
                .alongWith(new WaitCommand(liftBackTimeout)
                        .andThen(liftBack()).andThen(endClimbCommand()));
    }


    @Override
    public Command runAutoCommand() {
        startToBasket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(start), new Point(basket)))
                .setLinearHeadingInterpolation(start.getHeading(), basket.getHeading())
                .build();

        basketToGrab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basket), new Point(grab1)))
                .setLinearHeadingInterpolation(basket.getHeading(), grab1.getHeading())
                .build();

        grab1ToBasket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab1), new Point(basket)))
                .setLinearHeadingInterpolation(grab1.getHeading(), basket.getHeading())
                .build();

        basketToGrab2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basket), new Point(grab2)))
                .setLinearHeadingInterpolation(basket.getHeading(), grab2.getHeading())
                .build();

        grab2ToBasket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab2), new Point(basket)))
                .setLinearHeadingInterpolation(grab2.getHeading(), basket.getHeading())
                .build();

        basketToGrab3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basket), new Point(grab3)))
                .setLinearHeadingInterpolation(basket.getHeading(), grab3.getHeading())
                .build();

        grab3ToBasket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab3), new Point(basket)))
                .setLinearHeadingInterpolation(grab3.getHeading(), basket.getHeading())
                .build();

        basketToClimb = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basket), new Point(climb)))
                .setLinearHeadingInterpolation(basket.getHeading(), climb.getHeading())
                .build();

        basketToGrab4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basket), new Point(grab4)))
                .setLinearHeadingInterpolation(basket.getHeading(), grab4.getHeading())
                .build();

        grab4ToBasket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab4), new Point(basket)))
                .setLinearHeadingInterpolation(grab4.getHeading(), basket.getHeading())
                .build();

        return new SequentialCommandGroup(
                new InstantCommand(() -> follower.setStartingPose(start)),
                basket0(),
                basket1(),
                basket2(),
                basket3(),
                basket4(),
                climb()
        );
    }
}
