package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.grabAndBack;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.initialize;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.liftBack;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.liftToBasket;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.stowArmFromBasket;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.upLiftToBasket;

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
@Autonomous(name = "Basket 1+3", group = "Autos")
public class Basket1Plus3 extends LinearOpMode {
  AlphaLiftClaw liftClaw;
  Lift lift;
  AlphaSlide slide;

  public static Pose2dHelperClass start = new Pose2dHelperClass(-31.64, -65.06, 0.00);
  public static Pose2dHelperClass basket = new Pose2dHelperClass(-57.76, -58.25, 45.00);
  public static Pose2dHelperClass grab1 = new Pose2dHelperClass(-47.75, -54.43, 90.00);
  public static Pose2dHelperClass grab2 = new Pose2dHelperClass(-59.53, -54.43, 90.00);

  public static long waitDropTimeout = 200;



  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    TrajectorySequence startToBasket = drive.trajectorySequenceBuilder(start.toPose2d())
            .lineToSplineHeading(basket.toPose2d())
            .build();

    TrajectorySequence basketToGrab1 = drive.trajectorySequenceBuilder(basket.toPose2d())
            .lineToSplineHeading(grab1.toPose2d())
            .build();

    TrajectorySequence grab1ToBasket = drive.trajectorySequenceBuilder(grab1.toPose2d())
            .lineToSplineHeading(basket.toPose2d())
            .build();

    TrajectorySequence basketToGrab2 = drive.trajectorySequenceBuilder(basket.toPose2d())
            .lineToSplineHeading(grab2.toPose2d())
            .build();

    TrajectorySequence grab2ToBasket = drive.trajectorySequenceBuilder(grab2.toPose2d())
            .lineToSplineHeading(basket.toPose2d())
            .build();



//    TrajectorySequence trajTotal = drive.trajectorySequenceBuilder(new Pose2d(-24.23, -62.40, Math.toRadians(90.00)))
//            .lineToConstantHeading(new Vector2d(-2.97, -33.37))
//            .lineToSplineHeading(new Pose2d(-56.91, -56.69, Math.toRadians(45.00)))
//            .lineToSplineHeading(new Pose2d(-49.60, -48.91, Math.toRadians(90.00)))
//            .lineToSplineHeading(new Pose2d(-56.91, -56.69, Math.toRadians(45.00)))
//            .lineToSplineHeading(new Pose2d(-60.34, -49.14, Math.toRadians(90.00)))
//            .lineToSplineHeading(new Pose2d(-56.91, -56.69, Math.toRadians(45.00)))
//            .lineToSplineHeading(new Pose2d(-48.23, -48.23, Math.toRadians(135.00)))
//            .lineToSplineHeading(new Pose2d(-56.91, -56.46, Math.toRadians(45.00)))
//            .lineToSplineHeading(new Pose2d(-24.46, 2.29, Math.toRadians(0.00)))
//            .build();


    // spotless:off
    CommandScheduler.getInstance()
        .schedule(
            new SequentialCommandGroup(
                initialize(liftClaw, slide),
                new InstantCommand(() -> drive.setPoseEstimate(startToBasket.start())),
                new AutoDriveCommand(drive, startToBasket),
                liftToBasket(liftClaw, lift),
                new WaitCommand(waitDropTimeout),
                liftBack(liftClaw, lift),
                new AutoDriveCommand(drive, basketToGrab1),
                grabAndBack(liftClaw, slide),
                new AutoDriveCommand(drive, grab1ToBasket),
                liftToBasket(liftClaw, lift),
                new WaitCommand(waitDropTimeout),
                liftBack(liftClaw, lift),
                new AutoDriveCommand(drive, basketToGrab2),
                grabAndBack(liftClaw, slide),
                new AutoDriveCommand(drive, grab2ToBasket),
                liftToBasket(liftClaw, lift),
                new WaitCommand(waitDropTimeout),
                liftBack(liftClaw, lift)
            ));

    // spotless:on
    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
      lift.periodicTest();
    }
  }
}
