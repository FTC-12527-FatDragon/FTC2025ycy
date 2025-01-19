package org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@TeleOp(group = "drive")
public class AvgTurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static long TURNS = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double sumError = 0, target = ANGLE, stdDeviation = 0, lastPos = 0;

        waitForStart();

        if (isStopRequested()) return;

        for (long i = 1; i <= TURNS; ++i) {
            drive.turn(Math.toRadians(ANGLE));
            double error = (Math.toDegrees(drive.getPoseEstimate().getHeading()) - lastPos - ANGLE) % 360;
            lastPos = Math.toDegrees(drive.getPoseEstimate().getHeading());
            telemetry_M.addData("ErrorPerTurn", error);
            telemetry_M.addData("Pose", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry_M.update();
            stdDeviation += Math.pow(error, 2);
            target = (target + 180) % 180 - 180;
            sumError += Math.abs(error);
        }
        sumError /= TURNS;
        stdDeviation = Math.sqrt(stdDeviation) / TURNS;
        telemetry_M.addData("AvgHeadingError", sumError);
        telemetry_M.addData("StdDeviation", stdDeviation);
        telemetry_M.update();
    }
}
