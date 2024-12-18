package org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.GeomUtil;
import org.firstinspires.ftc.teamcode.lib.Units;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

  public static void drawRobot(Canvas c, Pose2d t) {
    final double ROBOT_RADIUS = 9;

    c.setStrokeWidth(1);
    c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

    Vector2d halfv = new Vector2d(1, 0).rotated(t.getHeading()).times(0.5 * ROBOT_RADIUS);
    Vector2d p1 = new Vector2d(t.getX(), t.getY()).plus(halfv);
    Vector2d p2 = p1.plus(halfv);
    c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
  }

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    MecanumDrive drive = new MecanumDrive(hardwareMap);

    waitForStart();

    while (opModeIsActive()) {
      drive.moveRobotFieldRelative(
          -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
      if (gamepad1.left_stick_button) {
        drive.reset();
      }


      Pose2d pose = GeomUtil.toPose2d(drive.getPose());
      telemetry.addData("x", pose.getX());
      telemetry.addData("y", pose.getY());
      telemetry.addData("xmm", Units.inchesToMm(pose.getX()));
      telemetry.addData("ymm", Units.inchesToMm(pose.getY()));
      telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
      telemetry.update();

      TelemetryPacket packet = new TelemetryPacket();
      packet.fieldOverlay().setStroke("#3F51B5");
      drawRobot(packet.fieldOverlay(), pose);
      FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
  }
}
