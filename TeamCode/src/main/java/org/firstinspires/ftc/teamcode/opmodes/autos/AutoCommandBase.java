package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw.LiftArm_Handoff2BackwardGrabDelay;
import static org.firstinspires.ftc.teamcode.subsystems.AlphaSlide.slideRetractAuto;
import static org.firstinspires.ftc.teamcode.subsystems.AlphaSlide.slideRetractFar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.ParallelRaceGroup;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;
import org.firstinspires.ftc.teamcode.utils.RoadRunnerPose.BooleanArea;
import org.firstinspires.ftc.teamcode.utils.RoadRunnerPose.PoseArea;
import org.firstinspires.ftc.teamcode.utils.RoadRunnerPose.RectangularArea;

public abstract class AutoCommandBase extends LinearOpMode {
  public static long lift2BasketTimeout = 800;
//  public static long slideServoResponseTime = 600;
  public static long basketTimeout = 600;
  public static long handoffTimeout = 100;
  public static long tempTimeout = 1000;

  public static boolean telemetryInDashboard = true;

  public static long Grab2PreHangDelay = 0;
  public static long ChamberUpOffsetMs = -300;

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

  public static Command handoff(AlphaSlide slide, AlphaLiftClaw liftClaw ) {

    return slide
        .handoffCommand()
        .alongWith(liftClaw.openClawCommand())
        .andThen(liftClaw.closeClawCommand())
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }
  public Command handoff(){
    return (handoff(slide, liftClaw));
  }

  public static Command toPreHang(Lift lift, AlphaLiftClaw liftClaw) {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::chamberWrist),
        new InstantCommand(liftClaw::chamberLiftArm),
        new WaitCommand(100).andThen(lift.setGoalCommand(Lift.Goal.PRE_HANG))
    );
  }

  public Command toPreHang() {
    return toPreHang(lift, liftClaw);
  }

  public Command grabToPreHang(){
    return liftClaw.closeClawCommand().andThen(toPreHang());
  }

  public static Command upToChamber(Lift lift) {
    return new ParallelRaceGroup(
            new WaitCommand(500),
            lift.setGoalCommand(Lift.Goal.HANG)
    );
  }
  public Command upToChamber(){
    return upToChamber(lift);
  }

//  public Command chamberOpenClaw() {
//    return new InstantCommand(liftClaw::openClaw);
//  }

  public Command chamberToGrab() {
    return chamberToGrab(lift, liftClaw).alongWith(liftClaw.openClawCommand()); // To override behavior used in teleop.
  }

  public static Command chamberToGrab(Lift lift, AlphaLiftClaw liftClaw) {
    return new ParallelCommandGroup(
            new InstantCommand(liftClaw::grabWrist),
            new InstantCommand(liftClaw::grabLiftArm).andThen(new WaitCommand(LiftArm_Handoff2BackwardGrabDelay)).andThen(liftClaw.openClawCommand()),
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

  public Command endClimbCommand() {
    return new ParallelCommandGroup(
         new InstantCommand(liftClaw::climbArm),
         new InstantCommand(liftClaw::basketWrist)
    );
  }


  public static Command liftToBasket(Lift lift, AlphaLiftClaw liftClaw) {
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

  public Command liftToBasket(long delay) {
    return new SequentialCommandGroup(
            new ParallelRaceGroup(
                    new WaitCommand(lift2BasketTimeout),
                    lift.setGoalCommand(Lift.Goal.BASKET)
            ),
            new InstantCommand(liftClaw::upLiftArm)
                    .alongWith(new InstantCommand(liftClaw::basketWrist)),
            new WaitCommand(basketTimeout + delay),
            new InstantCommand(liftClaw::openClaw)

    );
  }

  public Command liftToBasket(){
    return liftToBasket(lift,liftClaw);
  }

  public static Command liftBack(Lift lift, AlphaLiftClaw liftClaw) {
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

  public Command liftBack(){
    return liftBack(lift, liftClaw);
  }


  public Command grabAndBack() {
    return new SequentialCommandGroup(
            new InstantCommand(slide::autoForwardSlideExtension),
            new WaitCommand(slideRetractFar),
            slide.autoGrabCommand(),
            slide.handoffCommand(),
            liftClaw.closeClawCommand(),
            new WaitCommand(handoffTimeout),
            slide.autoOpenIntakeClaw()
    );
  }

  public Command grabAndBack3() {
    return new SequentialCommandGroup(
            new InstantCommand(slide::autoForwardSlideExtension),
            new WaitCommand(slideRetractFar),
            slide.autoGrabCommand3(),
            slide.handoffCommand(),
            liftClaw.closeClawCommand(),
            new WaitCommand(handoffTimeout),
            slide.autoOpenIntakeClaw()
    );
  }

  public Command forwardslideCommand() {
    return new SequentialCommandGroup(
            new InstantCommand(slide::autoForwardSlideExtension),
            new WaitCommand(slideRetractFar)
    );
  }

  public Command grabWithoutForwardAndBack() {
    return new SequentialCommandGroup(
            slide.autoGrabCommand(),
            slide.handoffCommand(),
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
   * Follow a trajectory
   * @param toFollow The trajectory to follow
   * @return The command to follow
   */
  protected Command drive(TrajectorySequence toFollow) {
    return new AutoDriveCommand(drive, toFollow);
  }

  private static final BooleanArea FalseArea = new BooleanArea(false);

  /**
   * Follow a trajectory with your custom timeout.
   * @param toFollow The trajectory to follow
   * @param admissibleTimeout Your timeout, note that this timeout can only be smaller than ADMISSIBLE_TIMEOUT in SampleMecanumDrive.
   * @return The command to follow
   */
  protected Command drive(TrajectorySequence toFollow, double admissibleTimeout){
    return drive(toFollow, admissibleTimeout, FalseArea);
  }

  /**
   * Follow a trajectory with your custom finish location.
   * @param toFollow The trajectory to follow
   * @param finishAt The area in which can be considered as finished
   * @return The command to follow
   * @apiNote ADMISSIBLE_TIMEOUT are also applied here, so in some case even it doesn't goes to finishAt it will stop.
   */
  protected Command drive(TrajectorySequence toFollow, PoseArea finishAt){
    return drive(toFollow, Double.MAX_VALUE, finishAt);
  }

  /**
   * Follow a trajectory, ends at either at your custom finish location or the timeout has expired.
   * @param toFollow The trajectory to follow
   * @param admissibleTimeout Your timeout, note that this timeout can only be smaller than ADMISSIBLE_TIMEOUT in SampleMecanumDrive.
   * @param finishAt The area in which can be considered as finished
   * @return The command to follow
   * @see #drive(TrajectorySequence, PoseArea)
   * @see #drive(TrajectorySequence, double)
   */
  protected Command drive(TrajectorySequence toFollow, double admissibleTimeout, PoseArea finishAt){
    return new ParallelRaceGroup(
            drive(toFollow),
            new WaitCommand((long)((toFollow.duration()+admissibleTimeout)*1000)),
            new WaitUntilCommand(() -> finishAt.covers(drive.getPoseEstimate()))
    );
  }

  /**
   * The cycle to hang specimen from observation zone grab to hang complete and stow lift
   * @param toChamberSequence The trajectory to follow to move robot to chamber
   * @param chamberToGrab The trajectory to follow to move robot back to grab position to grab the next specimen
   * @param chamberToGrabAdmissibleTimeout Admissible timeout of chamberToGrab
   * @return The command running these actions
   */
  public Command observationToChamberCycle(TrajectorySequence toChamberSequence, TrajectorySequence chamberToGrab, double chamberToGrabAdmissibleTimeout){
    return observationToChamberCycle(toChamberSequence, chamberToGrab, chamberToGrabAdmissibleTimeout, chamberToGrab());
  }

  public Command observationToChamberCycle(TrajectorySequence toChamberSequence, TrajectorySequence chamberToGrab, double chamberToGrabAdmissibleTimeout, Command chamberToGrabCommand){
    // NOTE: This function was shared by Chamber 1+3 and Chamber 1+4, be careful when modifying the code.
    return new SequentialCommandGroup(
            liftClaw.closeClawCommand(),

            drive(toChamberSequence, 0/*, new RectangularArea(new Vector2d(0, -24), 26.5, 2)*/) // No auto collaborate needed
                    .alongWith(new WaitCommand(Grab2PreHangDelay).andThen(toPreHang()))
                    .alongWith(new WaitCommand((long)(toChamberSequence.duration()*1000)+ChamberUpOffsetMs).andThen(upToChamber())),

            drive(chamberToGrab, chamberToGrabAdmissibleTimeout)
                    .alongWith(chamberToGrabCommand)
    );
  }

  /**
   * @see #observationToChamberCycle(TrajectorySequence, TrajectorySequence, double)
   */
  public Command observationToChamberCycle(TrajectorySequence toChamberSequence, TrajectorySequence chamberToGrab){
    return observationToChamberCycle(toChamberSequence, chamberToGrab, 1);
  }

  /**
   * 对CommandScheduler.getInstance().schedule的简写，功能一样。
   * @param commands Command to run
   * @see CommandScheduler#schedule(Command...)
   */
  protected void schedule(Command ...commands){
    CommandScheduler.getInstance().schedule(commands);
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
    drive = new SampleMecanumDrive(hardwareMap);
//    telemetry.setAutoClear(false); // FTC Dashboard does not support this, so set it separately.
    if (telemetryInDashboard) {
      telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }else telemetry_M = telemetry;
    telemetry_M.setMsTransmissionInterval(TelemetryTransmissionIntervalMs);
    CommandScheduler.getInstance().reset();

    lift = new Lift(hardwareMap, telemetry_M);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry_M);
    slide = new AlphaSlide(hardwareMap, telemetry_M);

    telemetry_M.addData("Current Robot Pose", drive.getPoseEstimate());
    telemetry_M.update();


//    drive.setPoseEstimate(getStartPose());
    Command toRun = initialize().andThen(runAutoCommand());//.andThen(autoFinish());

    CommandScheduler.getInstance().schedule(toRun);

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      DriveConstants.robotTeleOpStartPose = new Pose2dHelperClass(drive.getPoseEstimate());
      telemetry_M.addData("Robot TeleOp Pose", DriveConstants.robotTeleOpStartPose.toPose2d());
      telemetry_M.addData("Robot Current Pose", drive.getPoseEstimate());
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
