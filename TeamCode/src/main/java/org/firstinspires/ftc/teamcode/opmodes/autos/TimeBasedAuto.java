// package org.firstinspires.ftc.teamcode.opmodes.autos;
//
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AlphaAutoCommand.*;
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.autoFinish;
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.grabToPreHang;
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.initialize;
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.initializeForce;
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.stowArmFromBasket;
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.upLiftToBasket;
// import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.upToChamber;
//
// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.arcrobotics.ftclib.command.CommandScheduler;
// import com.arcrobotics.ftclib.command.InstantCommand;
// import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
// import com.arcrobotics.ftclib.command.RunCommand;
// import com.arcrobotics.ftclib.command.SequentialCommandGroup;
// import com.arcrobotics.ftclib.command.WaitCommand;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
// import org.firstinspires.ftc.teamcode.R;
// import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
// import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
// import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
// import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
// import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
// import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
// import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
// import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
// import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;
//
//
// @Autonomous(name="timeBasedAuto", group="autos")
// public class TimeBasedAuto extends LinearOpMode {
//    AlphaLiftClaw liftClaw;
//    AlphaLift lift;
//    AlphaSlide slide;
//
//    public void runOpMode() throws InterruptedException {
//        CommandScheduler.getInstance().reset();
//
//        lift = new AlphaLift(hardwareMap, telemetry);
//        liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
//        slide = new AlphaSlide(hardwareMap, telemetry);
//        MecanumDrive drive = new MecanumDrive(hardwareMap);
//
//
//        CommandScheduler.getInstance()
//                .schedule(
//                        new SequentialCommandGroup(
//
//
//                                initializeForce(liftClaw, slide),
//
//                                new WaitCommand(0),
//
//                                grabToPreHang(lift, liftClaw),
//                                new RunCommand(() -> drive.moveRobot(0.3, 0, 0))
//                                        .withTimeout(1500),
//
//                                new InstantCommand(() -> drive.moveRobot(0, 0, 0)),
//
//                                new WaitCommand(1000),
//                                upLiftToBasket(lift, liftClaw),
//                                new WaitCommand(500),
//                                stowArmFromBasket(lift, liftClaw),
//
//                                new WaitCommand(1000),
//                                new RunCommand(() -> drive.turnRobot(90, 1)),
//                                new WaitCommand(1000),
//                                new RunCommand(() -> drive.moveRobot(0.2, 0, 0))
//                                        .withTimeout(100),
//                                new RunCommand(() -> drive.moveRobot(0, 0, 0)),
//                                new RunCommand(() -> drive.moveRobot(0, 0.5, 0))
//                                        .withTimeout(2250),
//                                new RunCommand(() -> drive.moveRobot(0, 0, 0))
//
//                        )
//                );
//        waitForStart();
//        while (opModeIsActive() && !isStopRequested()) {
//            CommandScheduler.getInstance().run();
//        }
//    }
// }
