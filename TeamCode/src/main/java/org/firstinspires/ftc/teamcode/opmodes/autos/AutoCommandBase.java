package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.ParallelRaceGroup;

public abstract class AutoCommandBase extends LinearOpMode {
  public static long lift2BasketTimeout = 800;
  public static long slideServoResponseTime = 600;
  public static long basketTimeout = 600;
  public static long handoffTimeout = 100;
  public static long tempTimeout = 1000;

  public static boolean telemetryInDashboard = true;

  public static long Grab2ChamberUpDelay = 0;

  public static int TelemetryTransmissionIntervalMs = 50;

  protected Lift lift;
  protected AlphaLiftClaw liftClaw;
  protected AlphaSlide slide;
  protected SampleMecanumDrive drive;

  protected Telemetry telemetry_M;

  public Command upLiftToBasket() {
    return new ParallelCommandGroup(
        lift.setGoalCommand(Lift.Goal.BASKET)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  public Command stowArmFromBasket() {
    return new SequentialCommandGroup(
        liftClaw.openClawCommand(),
        liftClaw.foldLiftArmCommand(),
        lift.setGoalCommand(Lift.Goal.STOW));
  }

  public Command handoff() {
    return slide
        .handoffCommand()
        .alongWith(liftClaw.openClawCommand())
        .andThen(liftClaw.closeClawCommand())
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  public static Command toPreHang(Lift lift, AlphaLiftClaw liftClaw) {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::chamberWrist),
        new InstantCommand(liftClaw::chamberLiftArm),
        lift.setGoalCommand(Lift.Goal.PRE_HANG)
    );
  }

  public Command toPreHang() {
    return toPreHang(lift, liftClaw);
  }

  public Command grabToPreHang(){
    return liftClaw.closeClawCommand().andThen(toPreHang());
  }

  public Command upToChamber() {
    return new ParallelRaceGroup(
            new WaitCommand(500),
            lift.setGoalCommand(Lift.Goal.HANG)
    );
  }

//  public Command chamberOpenClaw() {
//    return new InstantCommand(liftClaw::openClaw);
//  }

  public Command chamberToGrab() {
    return chamberToGrab(lift, liftClaw);
  }

  public static Command chamberToGrab(Lift lift, AlphaLiftClaw liftClaw) {
    return new ParallelCommandGroup(
            liftClaw.openClawCommand(),
            new InstantCommand(liftClaw::grabWrist),
            new InstantCommand(liftClaw::grabLiftArm),
            lift.setGoalCommand(Lift.Goal.GRAB, true)
    );
  }

  public Command initialize() {
    return new ParallelCommandGroup(
        new InstantCommand(liftClaw::initialize),
        new InstantCommand(liftClaw::stowWrist),
        liftClaw.foldLiftArmCommand(0),
        liftClaw.closeClawCommand(0),
        new InstantCommand(slide::initialize));
  }


  public Command liftToBasket() {
    return new SequentialCommandGroup(
            new ParallelRaceGroup(
                    new WaitCommand(lift2BasketTimeout),
                    lift.setGoalCommand(Lift.Goal.BASKET)
            ),
            new InstantCommand(liftClaw::upLiftArm)
                    .alongWith(new InstantCommand(liftClaw::basketWrist)),
            new WaitCommand(basketTimeout),
            new InstantCommand(liftClaw::openClaw)

    );
  }


  public Command liftBack() {
    return new ParallelCommandGroup(
            liftClaw.foldLiftArmCommand(),
            new WaitCommand(tempTimeout),
            new InstantCommand(liftClaw::stowWrist),
            new WaitCommand(basketTimeout),
            new ParallelRaceGroup(
                    new WaitCommand(lift2BasketTimeout),
                    lift.setGoalCommand(Lift.Goal.STOW)
            )
    );
  }


  public Command grabAndBack() {
    return new SequentialCommandGroup(
            new InstantCommand(slide::autoForwardSlideExtension),
            new WaitCommand(slideServoResponseTime),
            slide.autoGrabCommand(),
            new InstantCommand(slide::autoBackSlideExtension),
            new WaitCommand(slideServoResponseTime),
            liftClaw.closeClawCommand(),
            new WaitCommand(handoffTimeout),
            slide.autoOpenIntakeClaw()
    );
  }

  public Command grabAndBack3() {
    return new SequentialCommandGroup(

            slide.autoGrabCommand3A(),
            new InstantCommand(slide::autoForwardSlideExtension),
            new WaitCommand(slideServoResponseTime),
            slide.autoGrabCommand3B(),
            new InstantCommand(slide::autoBackSlideExtension),
            new WaitCommand(slideServoResponseTime),
            liftClaw.closeClawCommand(),
            new WaitCommand(handoffTimeout),
            slide.autoOpenIntakeClaw()
    );
  }

  public Command initializeForce() {
    return new ParallelCommandGroup(
        new InstantCommand(liftClaw::initialize),
        new InstantCommand(liftClaw::stowWrist),
        liftClaw.foldLiftArmCommand(0),
        new InstantCommand(liftClaw::closeClawCommand),
        slide.aimCommand());
  }

  public Command autoFinish() {
    return new SequentialCommandGroup(
        initialize().andThen(slide.aimCommand()).andThen(new WaitCommand(100)));
  }

  /**
   * The cycle to hang specimen from observation zone grab to hang complete and stow lift
   * @param toChamberSequence The trajectory to follow to move robot to chamber
   * @param chamberToGrab The trajectory to follow to move robot back to grab position to grab the next specimen
   * @return The command running these actions
   */
  public Command observationToChamberCycle(TrajectorySequence toChamberSequence, TrajectorySequence chamberToGrab){
    // NOTE: This function was shared by Chamber 1+3 and Chamber 1+4, be careful when modifying the code.
    return new SequentialCommandGroup(
            liftClaw.closeClawCommand(),

            new AutoDriveCommand(drive, toChamberSequence)
                    .alongWith(new WaitCommand(Grab2ChamberUpDelay).andThen(toPreHang())),

            upToChamber(),

            new AutoDriveCommand(drive, chamberToGrab)
                    .alongWith(chamberToGrab())
    );
  }

  /**
      * Gets the command to run in auto, this should be implemented in each auto.
      *
      * @return The command to run.
   */
  public abstract Command runAutoCommand();

//  /**
//   * Gets the robot starting pose in field coordinate or its respective coordinates.
//   *
//   * @return The start pose following RoadRunner's coordinate system.
//   */
//  public abstract Pose2d getStartPose();

  @Override
  public void runOpMode() throws InterruptedException {
//    telemetry.setAutoClear(false); // FTC Dashboard does not support this, so set it separately.
    if (telemetryInDashboard) {
      telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }else telemetry_M = telemetry;
    telemetry_M.setMsTransmissionInterval(TelemetryTransmissionIntervalMs);
    CommandScheduler.getInstance().reset();

    lift = new Lift(hardwareMap, telemetry_M);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry_M);
    slide = new AlphaSlide(hardwareMap, telemetry_M);

    drive = new SampleMecanumDrive(hardwareMap);

//    drive.setPoseEstimate(getStartPose());
    Command toRun = initialize().andThen(runAutoCommand());//.andThen(autoFinish());

    CommandScheduler.getInstance().schedule(toRun);

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      periodic();
    }

    CommandScheduler.getInstance().reset();
  }

  /**
   *  The periodic function to update variables repeatedly。
   */
  public void periodic() {
    lift.periodicTest();
    drive.update();
    CommandScheduler.getInstance().run();
    telemetry_M.update();
  }
}
