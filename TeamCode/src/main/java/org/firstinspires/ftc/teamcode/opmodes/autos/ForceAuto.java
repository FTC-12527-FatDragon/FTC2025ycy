package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AlphaAutoCommand.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;


@Autonomous(name="forceAuto", group="autos")
public class ForceAuto extends LinearOpMode {
    AlphaLiftClaw liftClaw;
    AlphaLift lift;
    AlphaSlide slide;

    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        lift = new AlphaLift(hardwareMap, telemetry);
        liftClaw = new AlphaLiftClaw(hardwareMap, telemetry);
        slide = new AlphaSlide(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        liftClaw.autoInitialize();
        liftClaw.closeClaw();
        slide.backwardSlideExtension();

        CommandScheduler.getInstance()
                .schedule(
                        new SequentialCommandGroup(

//                                new ParallelDeadlineGroup(new WaitCommand(3000),
//                                        new TeleopDriveCommand(
//                                                drive,
//                                                () -> 0.8,
//                                                () -> 0,
//                                                () -> 0,
//                                                () -> false,
//                                                () -> false
//                                        ), alphaUpLiftToChamber(lift, liftClaw)),
////
//                                new TeleopDriveCommand(
//                                        drive,
//                                        () -> 0,
//                                        () -> 0,
//                                        () -> 0,
//                                        () -> false,
//                                        () -> false
//                                ),

                                new RunCommand(() -> drive.moveRobot(0, 0.8, 0))
                                        .alongWith(alphaUpLiftToChamber(lift, liftClaw))
                                        .withTimeout(3000),

                                new InstantCommand(() -> drive.moveRobot(0, 0, 0)),
//
                                new InstantCommand(() -> {
                                    telemetry.addData("Stop", "Stop");
                                    telemetry.update();
                                }),
//
                                new WaitCommand(1000),
                                slide.aimCommand(),
                                new InstantCommand(() -> lift.setGoal(AlphaLift.Goal.HANG)),
                                new WaitCommand(500),
                                alphaHangAndStowLift(lift, liftClaw, slide)
                        )
                );
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
    }
}
