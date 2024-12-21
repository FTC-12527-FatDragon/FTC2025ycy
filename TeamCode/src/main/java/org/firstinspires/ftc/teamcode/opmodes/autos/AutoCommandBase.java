package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

import lombok.Getter;


/**
 * Layout:            Field Coordinate:
 * +-----+-----+-----+-----+-----+-----^ x
 * |  0  |  1  |  2  |  3  |  4  |  5  |
 * <-----+-----+-----+-----+-----+-----+ CCW+
 * y                                    O
 * Robot Coordinate:
 *
 * Center
 *
 */
@Config
public abstract class AutoCommandBase extends LinearOpMode {
  protected LiftClaw liftClaw;
  protected Lift lift;
  protected SlideSuperStucture slide;
  protected SampleMecanumDrive drive;
  protected Climber climb;

  public static long handoff_slide2LiftCloseDelayMs = 0;
  public static long handoff_liftClose2OpenIntakeDelayMs = 50;

  public static class FieldConfig{
    public double blockSideLengthInch = 24;
  }
  public static FieldConfig Field = new FieldConfig();

  public static class RobotConfig{
    public double robotBack2FrontXLenInch = 16.5;
    public double robotLeft2RightYLenInch = 15.5;
  }
  public static RobotConfig Robot = new RobotConfig();
//
//  public enum StartPoseConfig{
//    L0(robotCentral, true),
//    R0(robotCentral, false),
//    L1(robotCentral, true),
//    R1(robotCentral, false),
//    L2(robotCentral, true),
//    R2(robotCentral, false),
//    L3(robotCentral, true),
//    R3(robotCentral, false),
//    L4(robotCentral, true),
//    R4(robotCentral, false),
//    L5(robotCentral, true),
//    R5(new Vector2d(0, 0), false);
//    public final Vector2d startPose;
//    StartPoseConfig(final Vector2d startPose, boolean isLeft){
//      if(isLeft){
//        this.startPose = new Vector2d(startPose.getX(), startPose.getY() - robotCentral.getY());
//      }else{
//        this.startPose = startPose.plus(robotCentral);
//      }
//    }
//  }
//  public static StartPoseConfig StartPose = StartPoseConfig.L1;
//
//  public enum StartHeadingConfig{
//    LEFT(Math.toRadians(-90)),
//    FRONT(Math.toRadians(0)),
//    RIGHT(Math.toRadians(90)),
//    BACK(Math.toRadians(180));
//    public final double heading;
//    StartHeadingConfig(double headingrad){
//      heading = headingrad;
//    }
//  }
//  public static StartHeadingConfig StartHeading = StartHeadingConfig.FRONT;
//  protected static Vector2d robotCentral = new Vector2d(Robot.robotBack2FrontXLenInch, Robot.robotLeft2RightYLenInch);
//  protected static Pose2d startPose = new Pose2d(StartPose.startPose, StartHeading.heading);
  @Getter private static Pose2d autoEndPose = new Pose2d();

  protected void initialize() {
    initialize(true);
  }
  protected void initialize(boolean telemetryInDashboard){
    if(telemetryInDashboard){
      this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    CommandScheduler.getInstance().reset();
    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    climb = new Climber(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);

    drive = new SampleMecanumDrive(hardwareMap);

    slide.stow();
    slide.backwardSlideExtension();
    liftClaw.closeClaw();
    liftClaw.foldLiftArm();
//    drive.setPoseEstimate(startPose);
  }

  protected Command upLiftToBasket() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 300)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  protected Command followTrajectory(TrajectorySequence trajectorySequence) {
    return new AutoDriveCommand(drive, trajectorySequence);
  }

  protected Command stowArmFromBasket() {
    return new SequentialCommandGroup(
        new InstantCommand(liftClaw::openClaw),
        new WaitCommand(100),
        new InstantCommand(liftClaw::foldLiftArm),
        new WaitCommand(200),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  protected Command slowHandoff() {
    return slowHandoff(slide, liftClaw);
  }

  public static Command slowHandoff(SlideSuperStucture slide, LiftClaw liftClaw) {
    return slide
        .slowHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  protected Command fastHandoff() {
    return fastHandoff(slide, liftClaw);
  }

  public static Command fastHandoff(SlideSuperStucture slide, LiftClaw liftClaw) {
    return slide
        .fastHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
        .andThen(new InstantCommand(liftClaw::closeClaw))
        .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  protected Command upLiftToChamber() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 90)
            .andThen(new InstantCommand(liftClaw::upLiftArm)));
  }

  protected Command handoffAndLiftToChamber() {
    return fastHandoff()
        .andThen(new WaitCommand(150))
        .andThen(new InstantCommand(slide::slideArmDown))
        .andThen(new WaitCommand(150))
        .andThen(upLiftToChamber());
  }

  protected Command hangAndStowLift() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG))
            .alongWith(new InstantCommand(slide::slideArmDown)),
        new ParallelDeadlineGroup(new WaitCommand(100), new WaitUntilCommand(() -> lift.atHome(10)))
            .andThen(new WaitCommand(160).deadlineWith(
                    lift.manualResetCommand().alongWith(
                            new SequentialCommandGroup(
                                    new WaitCommand(100),
                                    new InstantCommand(() -> drive.setWeightedDrivePower(new Pose2d(1, 0, 0))),
                                    new WaitCommand(50)
                            )
                    )
            ))
            .andThen(new InstantCommand(() -> {
              drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
              liftClaw.openClaw();
            })),
        new WaitCommand(50),
        new InstantCommand(liftClaw::foldLiftArm),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public Command wait(SampleMecanumDrive drive, long ms) {
    return new ParallelDeadlineGroup(
            new WaitCommand(ms), new RunCommand(drive::update).interruptOn(this::isStopRequested));
  }

  public Command initializeCommand() {
    return new ParallelCommandGroup(
        //        new InstantCommand(slide::forwardSlideExtension),
        new InstantCommand(liftClaw::closeClaw),
        new InstantCommand(slide::slideArmUp),
        new InstantCommand(slide::wristUp),
        new InstantCommand(slide::openIntakeClaw));
  }

  public Command autoFinish() {
    return new ParallelCommandGroup(
        slide.aimCommand(),
        // TODO: needs discussion
        slide.manualResetCommand().withTimeout(1000), // interruptOn(slide::atHome),
        // lift.resetCommand().interruptOn(() -> lift.atHome(3)),
        lift.manualResetCommand().withTimeout(1000),
        new InstantCommand(liftClaw::openClaw),
        new InstantCommand(() -> autoEndPose = drive.getPoseEstimate()));
  }

  public abstract Command runAutoCommand();

  @Override
  public void runOpMode() throws InterruptedException {
    double origval = slide.IntakeClawServo_OPEN;
    initialize();
    slide.IntakeClawServo_OPEN = 0.7;
    Command toRun = runAutoCommand().andThen(autoFinish());
    waitForStart();

    CommandScheduler.getInstance().schedule(toRun);

    while (opModeIsActive() && !isStopRequested()) {
      lift.periodicTest();
      CommandScheduler.getInstance().run();
    }
    slide.IntakeClawServo_OPEN = origval;
  }
}
