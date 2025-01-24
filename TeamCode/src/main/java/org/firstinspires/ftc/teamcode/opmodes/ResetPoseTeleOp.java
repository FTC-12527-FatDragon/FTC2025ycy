package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;

@TeleOp(name = "Reset start pose", group = "drive")
public class ResetPoseTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveConstants.setRobotTeleOpStartPose(new Pose2d());
        telemetry.addLine("Start pose being reset (x: 0, y: 0, heading: 0)");
        telemetry.update();
    }
}
