package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AlphaAutoCommand.*;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.autoFinish;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.grabToPreHang;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.initialize;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.upToChamber;

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


        CommandScheduler.getInstance()
                .schedule(
                        new SequentialCommandGroup(

                                initialize(liftClaw, slide),

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

                                grabToPreHang(lift, liftClaw),
                                new RunCommand(() -> drive.moveRobot(0.55, 0, 0))
                                        .withTimeout(2250),

                                new InstantCommand(() -> drive.moveRobot(0, 0, 0)),
//
                                new InstantCommand(() -> {
                                    telemetry.addData("Stop", "Stop");
                                    telemetry.update();
                                }),
//
                                new WaitCommand(1000),
                                upToChamber(lift),
                                new WaitCommand(500),
                                hangAndStowLift(lift, liftClaw),

                                autoFinish(liftClaw, lift, slide)
                        )
                );
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
    }
}
