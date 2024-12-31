package org.firstinspires.ftc.teamcode.opmodes.autos;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStructure;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

@Config
@Autonomous(name = "Basket 1+3", group = "Autos")
public class Basket1Plus3 extends AutoCommandBase {
  public static boolean isAscent = true;

  // For Basket Scoring
  public static Pose2dHelperClass Basket = new Pose2dHelperClass(9.5, 16.5, -45);

  // The right sample
  public static Pose2dHelperClass S3 = new Pose2dHelperClass(23, 8.75, 0);

  // The middle sample
  public static Pose2dHelperClass S2 = new Pose2dHelperClass(23, 19.5, 0);

  // The left sample
  public static Pose2dHelperClass S1Extend = new Pose2dHelperClass(10, 19.5, 20);

  public static long basketWaitMs = 500;

  // Ascent zone
  public static double xValue5 = 60;
  public static double yValue5 = -16;
  public static double heading5 = 90;
  public static double tangent5 = -90;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

  // Start to Basket
  TrajectorySequence L12Basket =
      TrajectoryManager.trajectorySequenceBuilder(startPose)
          .splineToLinearHeading(Basket.toPose2d(), Basket.getHeadingRad())
          .build();

  // Basket to the rightmost sample
  TrajectorySequence Basket2S3 =
      TrajectoryManager.trajectorySequenceBuilder(L12Basket.end())
          .lineToLinearHeading(S3.toPose2d())
          .build();

  // rightmost sample to basket
  TrajectorySequence S32Basket =
      TrajectoryManager.trajectorySequenceBuilder(Basket2S3.end())
          .lineToLinearHeading(Basket.toPose2d())
          .build();

  // basket to middle sample
  TrajectorySequence Basket2S2 =
      TrajectoryManager.trajectorySequenceBuilder(S32Basket.end())
          .lineToLinearHeading(S2.toPose2d())
          .build();

  // middle sample to basket
  TrajectorySequence S22Basket =
      TrajectoryManager.trajectorySequenceBuilder(Basket2S2.end())
          .lineToLinearHeading(Basket.toPose2d())
          .build();

  // basket to leftmost sample
  TrajectorySequence Basket2S1Extend =
      TrajectoryManager.trajectorySequenceBuilder(S22Basket.end())
          .lineToLinearHeading(S1Extend.toPose2d())
          .build();

  // leftmost sample to basket
  TrajectorySequence S1Extend2Basket =
      TrajectoryManager.trajectorySequenceBuilder(Basket2S1Extend.end())
          .lineToLinearHeading(Basket.toPose2d())
          .build();

  //  // basket to ascent zone
  TrajectorySequence Basket2Ascent =
      TrajectoryManager.trajectorySequenceBuilder(S1Extend2Basket.end())
          .splineToLinearHeading(new Pose2d(xValue5, yValue5, Math.toRadians(heading5)), tangent5)
          .build();

  public Pose2d getStartPose() {
    return new Pose2d(); // TODO: return the field relative pose
  }

  @Override
  public Command runAutoCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> drive.setPoseEstimate(L12Basket.start()))
            .alongWith(slide.manualResetCommand().withTimeout(200)),
        slide.aimCommand().beforeStarting(liftClaw::closeClaw),
        followTrajectory(L12Basket).alongWith(upLiftToBasket()),
        wait(drive, 200),
        stowArmFromBasket(),
        followTrajectory(Basket2S3).alongWith(slide.aimCommand()),
        slide.grabCommand(),
        followTrajectory(S32Basket).alongWith(slowHandoff().andThen(upLiftToBasket())),
        wait(drive, basketWaitMs),
        stowArmFromBasket(),
        followTrajectory(Basket2S2).alongWith(slide.aimCommand()),
        slide.grabCommand(),
        followTrajectory(S22Basket).alongWith(slowHandoff().andThen(upLiftToBasket())),
        wait(drive, basketWaitMs),
        stowArmFromBasket(),
        followTrajectory(Basket2S1Extend).alongWith(slide.aimCommand()),
        //                wait(drive, 300),
        new InstantCommand(
            () -> {
              slide.forwardSlideExtension(440);
              slide.setServoPos(SlideSuperStructure.TurnServo.DEG_0);
            }),
        wait(drive, 500),
        slide.grabCommand(),
        //                wait(drive, 300),
        slowHandoff(),
        //                wait(drive, 300),
        followTrajectory(S1Extend2Basket),
        upLiftToBasket(),
        wait(drive, basketWaitMs),
        stowArmFromBasket(),
        //                wait(drive, 1500),
        isAscent
            ? followTrajectory(Basket2Ascent)
                .alongWith(climb.decline2ArmUp())
                .andThen(climb.elevate2ArmDown())
            : new InstantCommand(() -> {}));
  }
}
