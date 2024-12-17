package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
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
import java.util.HashMap;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@TeleOp(name = "ycyAlphaTeleop")
public class AlphaCar extends CommandOpMode {
  private GamepadEx gamepadEx1;
  private AlphaLift lift;
  private AlphaLiftClaw liftClaw;
  private AlphaSlide slide;
  private MecanumDrive drive;

  private boolean isPureHandoffComplete = false;
  private boolean isHangComplete = false;

  @Override
  public void initialize() {
    gamepadEx1 = new GamepadEx(gamepad1);

    lift = new AlphaLift(hardwareMap, telemetry);
    liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
    slide = new AlphaSlide(hardwareMap, telemetry);
    drive = new MecanumDrive(hardwareMap);

    slide.initialize();
    liftClaw.initialize();

    // Teleop Drive Command
    drive.setDefaultCommand(
        new TeleopDriveCommand(
            drive,
            () -> -gamepadEx1.getLeftY(),
            () -> -gamepadEx1.getLeftX(),
            () -> gamepadEx1.getRightX(),
            () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON),
            () -> gamepadEx1.getButton(GamepadKeys.Button.START)));

    // Basket Up Command
    gamepadEx1
        .getGamepadButton(GamepadKeys.Button.X)
        .whenPressed(
            new ConditionalCommand(
                new InstantCommand(),
                new ParallelCommandGroup(
                    new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.BASKET)),
                    new WaitUntilCommand(() -> lift.getCurrentPosition() > 600)
                        .andThen(new InstantCommand(liftClaw::upLiftArm))),
                () -> !isPureHandoffComplete));

    // Basket Drop and Back
    gamepadEx1
        .getGamepadButton(GamepadKeys.Button.B)
        .whenPressed(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(() -> slide.slideArmDown())
                        .andThen(new WaitCommand(100))
                        .andThen(new InstantCommand(() -> slide.setGoal(AlphaSlide.Goal.AIM))),
                    new InstantCommand(),
                    () -> lift.getGoal() == AlphaLift.Goal.HANG),
                new InstantCommand(liftClaw::openClaw),
                new InstantCommand(liftClaw::basketWrist),
                new WaitCommand(100),
                new InstantCommand(liftClaw::foldLiftArm),
                new WaitCommand(500),
                new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.STOW)),
                new InstantCommand(() -> isPureHandoffComplete = false)));

    // Aim
    gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(slide.aimCommand(), false);

    // Grab when aim
    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.A)
                    && slide.getGoal() == AlphaSlide.Goal.AIM)
        .whenPressed(slide.grabCommand(), false);

    // Pure Handoff
    Supplier<Command> handoffCommand =
        () ->
            slide
                .handoffCommand()
                .alongWith(new InstantCommand(liftClaw::openClaw))
                .andThen(new WaitCommand(500))
                .andThen(new InstantCommand(() -> liftClaw.closeClaw()))
                .andThen(new WaitCommand(250))
                .andThen(new InstantCommand(() -> slide.openIntakeClaw()));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
                    && slide.getGoal() == AlphaSlide.Goal.AIM
                    && lift.getGoal() == AlphaLift.Goal.STOW)
        .whenPressed(
            new ConditionalCommand(
                    new InstantCommand(() -> slide.preHandoffSlideExtension()),
                    new InstantCommand(),
                    () -> !slide.isSlideForward())
                .alongWith(
                    new InstantCommand(() -> slide.handoffWristTurn())
                        .alongWith(
                            handoffCommand
                                .get()
                                .andThen(new WaitCommand(50))
                                .andThen(new InstantCommand(() -> isPureHandoffComplete = true))),
                    new InstantCommand()),
            false);

    // Chamber Command from Grab
    Supplier<Command> preHang =
        () ->
            new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG))
                .alongWith(new InstantCommand(() -> liftClaw.chamberWrist()))
                .andThen(new InstantCommand(() -> liftClaw.chamberLiftArm()));

    Supplier<Command> grab =
        () ->
            new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.GRAB))
                .alongWith(new InstantCommand(() -> liftClaw.openClaw()))
                .andThen(new InstantCommand(() -> liftClaw.grabWrist()))
                .alongWith(new InstantCommand(() -> liftClaw.grabLiftArm()));

    Supplier<Command> stow =
        () ->
            new ConditionalCommand(
                new InstantCommand(),
                new InstantCommand(() -> liftClaw.openClaw())
                    .andThen(new InstantCommand(() -> liftClaw.foldLiftArm()))
                    .alongWith(new InstantCommand(() -> liftClaw.basketWrist()))
                    .andThen(new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.STOW)))
                    .andThen(new InstantCommand(() -> isHangComplete = false)),
                () -> !isHangComplete);

    SelectCommand specimenCommands =
        new SelectCommand(
            new HashMap<Object, Command>() {
              {
                put(AlphaLift.Goal.GRAB, preHang.get());
                put(AlphaLift.Goal.STOW, grab.get());
                put(AlphaLift.Goal.HANG, stow.get());
              }
            },
            () -> lift.getGoal());

    gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(specimenCommands, false);

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    && lift.getGoal() == AlphaLift.Goal.PRE_HANG)
        .whenPressed(
            new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.HANG))
                .andThen(new WaitCommand(300))
                .andThen(new InstantCommand(() -> isHangComplete = true)));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                    && lift.getGoal() == AlphaLift.Goal.HANG)
        .whenPressed(
            new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.PRE_HANG))
                .andThen(new WaitCommand(300))
                .andThen(new InstantCommand(() -> isHangComplete = false)));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                    && lift.getGoal() == AlphaLift.Goal.GRAB)
        .whenPressed(new InstantCommand(() -> liftClaw.switchLiftClaw()));

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
                    && lift.getGoal() == AlphaLift.Goal.STOW)
        .whenPressed(lift.resetCommand().withTimeout(100));

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                    && slide.getGoal() != AlphaSlide.Goal.HANDOFF)
        .whenPressed(new InstantCommand(slide::forwardSlideExtension));

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                    && slide.getGoal() != AlphaSlide.Goal.HANDOFF)
        .whenPressed(new InstantCommand(slide::backwardSlideExtension));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                    && slide.getGoal() == AlphaSlide.Goal.AIM)
        .whenPressed(new InstantCommand(() -> slide.leftTurnServo()));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                    && slide.getGoal() == AlphaSlide.Goal.AIM)
        .whenPressed(new InstantCommand(() -> slide.rightTurnServo()));
  }
}

/*package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.AlphaClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config @TeleOp(name = "ycyAlphaTeleOP")
public class AlphaCar extends LinearOpMode {
    public static boolean isHeadless = true;
    public static double maxPower = 1;
    public AlphaSlide slide;
    public AlphaLift lift;
//    private Servo clawServo, clawTurnServo, slideServo;
//    private clawTurnServoState turnState = clawTurnServoState.ORIGIN;
    private SampleMecanumDrive drive;
    private GamepadEx gamepadEx1;
    static{
        headless = new Gamepad.RumbleEffect.Builder().addStep(0.5, 0.1, 250)
                .addStep(0, 0.5, 250)
                .build();
        headnormal = new Gamepad.RumbleEffect.Builder().addStep(1, 1, 250)
                .addStep(0, 0, 250)
                .addStep(1, 1, 250)
                .build();
    }
    private static final Gamepad.RumbleEffect headless;
    private static final Gamepad.RumbleEffect headnormal;

    @Override
    public void runOpMode() throws InterruptedException {
        double position = 0;
        lift = new AlphaLift(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        ToggleBoolean isRightDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_right);
        ToggleBoolean isLeftDPadPressed = new ToggleBoolean(() -> gamepad1.dpad_left);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenHeld(
                new SequentialCommandGroup(
                        new InstantCommand(drive::resetHeading),
                        new WaitCommand(300),
                        new InstantCommand(() -> {
                            isHeadless=!isHeadless;
                            if(isHeadless){
                                gamepad1.runRumbleEffect(headless);
                            }else{
                                gamepad1.runRumbleEffect(headnormal);
                            }
                        }))
        );
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    if(maxPower==0.3){
                        maxPower = 1;
                    }else{
                        maxPower = 0.3;
                    }
                })
        );
        waitForStart();
        while(opModeIsActive()){
            if(isHeadless){
                drive.setFieldRelativeDrivePower(new Pose2d(
                        -gamepad1.left_stick_y * maxPower,
                        -gamepad1.left_stick_x * maxPower,
                        -gamepad1.right_stick_x * maxPower
                ));
            }else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * maxPower,
                                -gamepad1.left_stick_x * maxPower,
                                -gamepad1.right_stick_x *maxPower
                        )
                );
            }

            if(gamepad1.dpad_up) {
                lift.setOpenLoop(1);
//                lift.setSetPoint(0.5);
            }else if(gamepad1.dpad_down) {
                lift.setOpenLoop(-1);
//                lift.setSetPoint(0);
            }else lift.setOpenLoop(0);
            if(gamepad1.left_bumper) claw.aim(1, position);
            else if(gamepad1.right_bumper) claw.retract();

            boolean rightDPad = isRightDPadPressed.isPressed();

            if(rightDPad) {
                telemetry.addLine("RD Pressed");
                position += 1.0/12;
                claw.setYaw(position);
            }
            if(isLeftDPadPressed.isPressed()) {
                telemetry.addLine("LD Pressed");
                position -= 1.0/12;
                claw.setYaw(position);
            }

            if(gamepad1.x) {
                claw.release();
            }
            else if(gamepad1.y) {
                claw.grab();
            }
            telemetry.addData("forward",gamepad1.left_stick_y);
            telemetry.addData("fun",gamepad1.left_stick_x);
            telemetry.addData("turn",gamepad1.right_stick_x);
//            telemetry.addData("clawServo",clawServo.getPosition());
            telemetry.addData("heading",drive.getHeading());
            telemetry.addData("isRightDPadPressed",rightDPad);
            telemetry.addData("perv Button",isRightDPadPressed.getLastButton());
            telemetry.update();
            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();
    }
}*/
