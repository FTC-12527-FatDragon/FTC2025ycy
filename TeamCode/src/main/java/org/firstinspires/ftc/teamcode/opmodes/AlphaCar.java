package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import java.util.HashMap;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommandBase;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;

@Config
@TeleOp(name = "AlphaYCYTeleOp")
public class AlphaCar extends CommandOpMode {
//  private ColorSensor intakeClawSensor;
  private Climber climber;
  private GamepadEx gamepadEx1, gamepadEx2;
  private Lift lift;
  private AlphaLiftClaw liftClaw;
  private AlphaSlide slide;
  private SampleMecanumDrive drive;
  private Telemetry telemetry_M;

  private boolean isPureHandoffComplete = false;
  private boolean isHangComplete = false;
  private boolean shouldDisable = false;

  public static boolean isSetPose = true;
  public static double xPose = 0;
  public static double yPose = 0;
  public static double headingDegree = 0;

  private OSState currentState = OSState.Teleop;

  public enum OSState{
    Teleop,
    Halfauto;
  }
  //Trajectories
  public Pose2dHelperClass start = DriveConstants.getRobotTeleOpStartPose();
  public static Pose2dHelperClass basket =
          currentRobot == DriveConstants.RobotType.ALPHA ?
                  new Pose2dHelperClass(-56.76, -57.25, 45.00) :
                  new Pose2dHelperClass(-57.76, -58.25, 45.00);
  public static Pose2dHelperClass oberservationZone = new Pose2dHelperClass(51.5,-58.4,90);
  public static Pose2dHelperClass positionofSample = new Pose2dHelperClass(51.5, -65.29,90);
  public static double gap = 3;
  public static Pose2dHelperClass chamber = new Pose2dHelperClass(-7, -29, 90.00);
  public static Pose2dHelperClass chamber1 = new Pose2dHelperClass(chamber.X + gap, chamber.Y, 90.00);

  @Override
  public void initialize() {
    telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    gamepadEx1 = new GamepadEx(gamepad1);
    gamepadEx2 = new GamepadEx(gamepad2);

    gamepadEx1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
            new InstantCommand(() -> {
              switch (currentState) {
                case Teleop:
                  currentState = OSState.Halfauto;
                  break;
                case Halfauto:
                  currentState = OSState.Teleop;
                  break;
              }
            })
    );

    lift = new Lift(hardwareMap, telemetry_M);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry_M);
    slide = new AlphaSlide(hardwareMap, telemetry_M);
    drive = new SampleMecanumDrive(hardwareMap);
//    if (currentRobot==DriveConstants.RobotType.DELTA) {
//      intakeClawSensor = new ColorSensor(hardwareMap, "intakeClawSensor");
//    }
    if(currentRobot == DriveConstants.RobotType.EPSILON){
      climber = new Climber(hardwareMap);
    }
    slide.initialize();
    liftClaw.initialize();

    telemetry_M.addData("Current Robot Pose", DriveConstants.robotTeleOpStartPose.toPose2d());
//    telemetry_M.addData("Local Robot Pose", LocalRobotTeleOpStartPose.toPose2d());
    telemetry_M.update();
    drive.setPoseEstimate(DriveConstants.robotTeleOpStartPose.toPose2d());
//    lift.setGoal(Lift.Goal.STOW);

    // Teleop Drive Command
    drive.setDefaultCommand(
            new TeleopDriveCommand(
                    drive,
                    () -> gamepadEx1.getLeftY(),
                    () -> -gamepadEx1.getLeftX(),
                    () -> -gamepadEx1.getRightX(),
                    () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON),
                    () -> gamepadEx1.getButton(GamepadKeys.Button.START),
                    () -> currentState==OSState.Halfauto && drive.isBusy()));

    TrajectorySequence halfautoToBasket = drive.trajectorySequenceBuilder(DriveConstants.getRobotTeleOpStartPose().toPose2d())
            .lineToSplineHeading(basket.toPose2d())
            .build();

    TrajectorySequence halfautoToHumanPlayer = drive.trajectorySequenceBuilder(DriveConstants.getRobotTeleOpStartPose().toPose2d())
            .lineToSplineHeading(oberservationZone.toPose2d())
            .build();

    TrajectorySequence halfautoToChamber = drive.trajectorySequenceBuilder(DriveConstants.getRobotTeleOpStartPose().toPose2d())
            .lineToSplineHeading(chamber.toPose2d())
            .build();

    TrajectorySequence toGrab = drive.trajectorySequenceBuilder(DriveConstants.getRobotTeleOpStartPose().toPose2d())
            .lineToSplineHeading(positionofSample.toPose2d())
            .build();

    TrajectorySequence transPlace = drive.trajectorySequenceBuilder(DriveConstants.getRobotTeleOpStartPose().toPose2d())
            .lineToSplineHeading(chamber1.toPose2d())
            .build();
    // Basket Up Command
    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.X)
                            && currentState == OSState.Teleop)
            .whenPressed(
                    new ConditionalCommand(
                            new InstantCommand(),
                            new ParallelCommandGroup(
                                    slide.aimCommand(),
                                    lift.setGoalCommand(Lift.Goal.BASKET, false),
                                    new WaitUntilCommand(() -> lift.getCurrentPosition() > 600)
                                            .andThen(new InstantCommand(liftClaw::upLiftArm)
                                                    .alongWith(new InstantCommand(liftClaw::basketWrist)))),
                            () -> !isPureHandoffComplete));
//    gamepadEx1
//        .getGamepadButton(GamepadKeys.Button.X)
//        .whenPressed(
//            new ConditionalCommand(
//                new InstantCommand(),
//                new ParallelCommandGroup(
//                    slide.aimCommand(),
//                    lift.setGoalCommand(Lift.Goal.BASKET, false),
//                    new WaitUntilCommand(() -> lift.getCurrentPosition() > 600)
//                                .andThen(new InstantCommand(liftClaw::upLiftArm)
//                                .alongWith(new InstantCommand(liftClaw::basketWrist)))),
//                () -> !isPureHandoffComplete));
    //Teleop mode
    // Basket Drop and Back
    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.B)
                            && currentState == OSState.Teleop)
            .whenPressed(
                    new ConditionalCommand(
                            liftClaw.openClawCommand(),
                            new SequentialCommandGroup(
                                    liftClaw.openClawCommand(),
                                    new ParallelCommandGroup(
                                            new ConditionalCommand(
                                                    new InstantCommand(() -> slide.slideArmDown())
                                                            .andThen(new WaitCommand(100))
                                                            .andThen(new InstantCommand(() -> slide.setGoal(AlphaSlide.Goal.AIM))),
                                                    new InstantCommand(),
                                                    () -> lift.getGoal() == Lift.Goal.HANG),
                                            liftClaw.foldLiftArmCommand()
                                    ),
                                    new InstantCommand(liftClaw::stowWrist),
                                    new WaitCommand(100),
                                    lift.setGoalCommand(Lift.Goal.STOW, false),
                                    new InstantCommand(() -> isPureHandoffComplete = false)),
                            () -> false//liftClaw::getLiftClawPos
                    ));


    // Aim
    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.Y)
    )
            .whenPressed(slide.aimCommand(), false);


    // Grab when aim
    gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(slide.openClawCommand()
                    .andThen(slide.grabCommand()), false);


    // Pure Handoff
    Supplier<Command> handoffCommand =
            () ->
                    slide
                            .handoffCommand()
                            .alongWith(liftClaw.openClawCommand())
                            .andThen(new WaitCommand(60),liftClaw.closeClawCommand())
                            .andThen(new InstantCommand(() -> slide.openIntakeClaw()));


    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && slide.getGoal() == AlphaSlide.Goal.STOW
                            && lift.getGoal() == Lift.Goal.STOW
                            && currentState == OSState.Teleop)
            .whenPressed(
                    new ConditionalCommand(
                            new InstantCommand(() -> slide.preHandoffSlideExtension()),
                            new InstantCommand(),
                            () -> !slide.isSlideForward())
                            .alongWith(
                                    new InstantCommand(() -> slide.handoffWristTurn())
                                            .alongWith(
                                                    handoffCommand.get()
                                                            .andThen(new WaitCommand(50))
                                                            .andThen(new InstantCommand(() -> isPureHandoffComplete = true)))),
                    false);

    // Chamber Command from Grab
    Command preHang = AutoCommandBase.toPreHang(lift, liftClaw);

    Command grab = AutoCommandBase.chamberToGrab(lift, liftClaw).alongWith(slide.aimCommand());

    Supplier<Command> stow =
            () ->
                    new ConditionalCommand(
                            new InstantCommand(),
                            liftClaw.openClawCommand(0)
                                    .andThen(liftClaw.foldLiftArmCommand(0))
                                    .alongWith(new InstantCommand(() -> liftClaw.stowWrist()))
                                    .andThen(lift.setGoalCommand(Lift.Goal.STOW, false))
                                    .andThen(new InstantCommand(() -> isHangComplete = false)),
                            () -> !isHangComplete);

    SelectCommand specimenCommands =
            new SelectCommand(
                    new HashMap<Object, Command>() {
                      {
                        put(Lift.Goal.GRAB, preHang);
                        put(Lift.Goal.STOW, grab);
                        put(Lift.Goal.HANG, stow.get());
                      }
                    },
                    () -> lift.getGoal());

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) &&
                            lift.getGoal() != Lift.Goal.BASKET && currentState == OSState.Teleop)
            .whenPressed(specimenCommands, false);
    //gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(specimenCommands, false);

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                            && lift.getGoal() == Lift.Goal.PRE_HANG && currentState == OSState.Teleop)
            .whenPressed(
                    lift.setGoalCommand(Lift.Goal.HANG)
                            .andThen(new InstantCommand(() -> isHangComplete = true)));

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                            && lift.getGoal() == Lift.Goal.HANG && currentState == OSState.Teleop)
            .whenPressed(
                    lift.setGoalCommand(Lift.Goal.PRE_HANG)
                            .andThen(new InstantCommand(() -> isHangComplete = false)));

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                            && lift.getGoal() == Lift.Goal.GRAB && currentState == OSState.Teleop)
            .whenPressed(liftClaw.switchLiftClawCommand());

    //        // Handoff from Aim
    //        // Chamber Command
    //        new FunctionalButton(
    //                () ->
    //                        gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
    //                                && slide.getGoal() == AlphaSlide.Goal.AIM)
    //                .whenPressed(
    //                        handoffCommand
    //                                .get()
    //                                .andThen(new WaitCommand(200))
    //                                .andThen(new InstantCommand(() -> slide.wristUp()))
    //                                .andThen(new WaitCommand(200))
    //                                .andThen(
    //                                        new ParallelCommandGroup(
    //                                                new InstantCommand(() ->
    // lift.setGoal(AlphaLift.Goal.PRE_HANG)),
    //                                                new WaitUntilCommand(() ->
    // lift.getCurrentPosition() > 200)
    //                                                        .andThen(new
    // InstantCommand(liftClaw::upLiftArm)))),
    //                        false);
    //
    //        new FunctionalButton(
    //                () ->
    //                        gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
    //                                && lift.getGoal() == AlphaLift.Goal.HANG)
    //                .whenPressed(new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG)));
    //
    //        new FunctionalButton(
    //                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) &&
    // isPureHandoffCompelte)
    //                .whenPressed(
    //                        new InstantCommand(() -> slide.wristUp())
    //                                .andThen(new WaitCommand(200))
    //                                .andThen(
    //                                        new ParallelCommandGroup(
    //                                                new InstantCommand(() ->
    // lift.setGoal(AlphaLift.Goal.PRE_HANG)),
    //                                                new WaitUntilCommand(() ->
    // lift.getCurrentPosition() > 200)
    //                                                        .andThen(new
    // InstantCommand(liftClaw::upLiftArm)))),
    //                        false);

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                            && lift.getGoal() == Lift.Goal.STOW
                            && currentState == OSState.Teleop)
            .whenHeld(
                    lift.manualResetCommand()
            );

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                            && currentState == OSState.Teleop
                            && currentRobot != DriveConstants.RobotType.ALPHA
                            && (slide.getSlideServo() == AlphaSlide.SlideServo.BACK
                            || slide.getSlideServo() == AlphaSlide.SlideServo.HANDOFF))
            .whenHeld(
                    new InstantCommand(() -> slide.setSlideServo(AlphaSlide.SlideServo.BACK)).andThen(
                            slide.manualResetCommand()
                    )
            );

    new FunctionalButton(
            () ->
                    gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                            && slide.getGoal() != AlphaSlide.Goal.HANDOFF
    )
            .whenPressed(new InstantCommand(slide::forwardSlideExtension));

    new FunctionalButton(
            () ->
                    gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                            && slide.getGoal() != AlphaSlide.Goal.HANDOFF)
            .whenPressed(new InstantCommand(slide::backwardSlideExtension));

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            && slide.getGoal() == AlphaSlide.Goal.AIM && currentState == OSState.Teleop)
            .whenPressed(new InstantCommand(() -> slide.leftTurnServo()));

    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                            && slide.getGoal() == AlphaSlide.Goal.AIM && currentState == OSState.Teleop)
            .whenPressed(new InstantCommand(() -> slide.rightTurnServo()));
    // Halfauto part
    // Halfauto basket
    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
                            && slide.getGoal() == AlphaSlide.Goal.AIM && currentState == OSState.Halfauto)
            .whenPressed( new InstantCommand(() -> shouldDisable = true)
                    .alongWith( new AutoDriveCommand(drive, halfautoToBasket))
                    .alongWith(AutoCommandBase.handoff(slide,liftClaw)
                            .andThen(AutoCommandBase.liftToBasket(lift,liftClaw)))
                    .andThen(new WaitCommand(200))
                    .andThen(AutoCommandBase.liftBack(lift, liftClaw))
                    .alongWith(new InstantCommand(()-> shouldDisable = false))
            );
    // Halfauto Chamber
    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.X)
                            && slide.getGoal() == AlphaSlide.Goal.AIM && currentState == OSState.Halfauto)
            .whenPressed(
                    new InstantCommand(() -> shouldDisable = true)
                            .alongWith(new AutoDriveCommand(drive,halfautoToHumanPlayer))
                            .alongWith(AutoCommandBase.handoff(slide,liftClaw))
                            .andThen(AutoCommandBase.chamberToGrab(lift, liftClaw))
                            .alongWith(new InstantCommand(()-> shouldDisable = false))
            );
    new FunctionalButton(
            () ->
                    gamepadEx1.getButton(GamepadKeys.Button.B)
                            && slide.getGoal() == AlphaSlide.Goal.AIM && currentState == OSState.Halfauto)
            .whenPressed(
                    new InstantCommand(()-> shouldDisable = true)
                            .andThen(new AutoDriveCommand(drive,toGrab))
                            .andThen(new AutoDriveCommand(drive,halfautoToChamber))
                            .alongWith(AutoCommandBase.toPreHang(lift,liftClaw))
                            .andThen(new AutoDriveCommand(drive,transPlace))
                            .andThen(AutoCommandBase.upToChamber(lift))
                            .andThen(new WaitCommand(300))
                            .andThen(AutoCommandBase.liftBack(lift,liftClaw))
                            .andThen(new AutoDriveCommand(drive,halfautoToHumanPlayer)
                                    .alongWith(AutoCommandBase.handoff(slide,liftClaw))
                                    .andThen(AutoCommandBase.chamberToGrab(lift, liftClaw)))
                            .alongWith(new InstantCommand(()-> shouldDisable = false))
            );

    if(climber!=null){
      new FunctionalButton(
              () ->
                      gamepadEx2.getButton(GamepadKeys.Button.DPAD_UP) &&
                              currentRobot == DriveConstants.RobotType.EPSILON)
              .whenHeld(
                      climber.elevateCommand()
              );

      new FunctionalButton(
              () ->
                      gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN) &&
                              currentRobot == DriveConstants.RobotType.EPSILON)
              .whenHeld(
                      climber.declineCommand()
              );

    }
    }

  @Override
  public void run() {
//    if (currentRobot==DriveConstants.RobotType.DELTA) {
//      telemetry_M.addData("blueValue", intakeClawSensor.getColor().blue);
//    }
    CommandScheduler.getInstance().run();
    lift.periodicTest();
    telemetry_M.update();
    TelemetryPacket packet = new TelemetryPacket();
    packet.fieldOverlay().setStroke("#3F51B5");
    LocalizationTest.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
    FtcDashboard.getInstance().sendTelemetryPacket(packet);

    if(isSetPose) {
      drive.setPoseEstimate(new Pose2d(xPose, yPose, Math.toRadians(headingDegree)));
      isSetPose = false;
    }
  }
}