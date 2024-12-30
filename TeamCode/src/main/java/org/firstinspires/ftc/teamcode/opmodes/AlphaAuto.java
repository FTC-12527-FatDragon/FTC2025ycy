//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.subsystems.AlphaLift;
//import org.firstinspires.ftc.teamcode.subsystems.AlphaLiftClaw;
//import org.firstinspires.ftc.teamcode.subsystems.AlphaSlide;
//import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
//
//@Autonomous(name = "AlphaAuto")
//public class AlphaAuto extends LinearOpMode {
//  private SampleMecanumDrive drive;
//  private AlphaLift lift;
//  private AlphaLiftClaw liftClaw;
//  private AlphaSlide slide;
//
//  @Override
//  public void runOpMode() throws InterruptedException {
//    drive = new SampleMecanumDrive(hardwareMap);
//    Pose2d startPoint = new Pose2d(0, 0);
//    Trajectory tra1 =
//        drive
//            .trajectoryBuilder(startPoint)
//            .splineTo(new Vector2d(30, 0), Math.toRadians(90))
//            .splineTo(new Vector2d(30, 30), Math.toRadians(180))
//            .splineTo(new Vector2d(0, 30), Math.toRadians(270))
//            .splineTo(new Vector2d(0, 0), Math.toRadians(0))
//            .build();
//    //        Trajectory tra2 = drive.trajectoryBuilder(tra1.end())
//    //                .lineToConstantHeading(new Vector2d(0,0)).build();
//    waitForStart();
//    drive.followTrajectoryAsync(tra1);
//    //        drive.followTrajectory(tra2);
//    while (opModeIsActive() && !isStopRequested()) {
//      drive.update();
//    }
//  }
//}
