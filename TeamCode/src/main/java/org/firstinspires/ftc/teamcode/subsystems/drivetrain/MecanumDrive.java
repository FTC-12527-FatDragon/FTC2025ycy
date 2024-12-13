package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;

@Deprecated
public class MecanumDrive extends SubsystemBase {
  private final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
  private final GoBildaPinpointDriver od;
  private double yawOffset;

  public MecanumDrive(final HardwareMap hardwareMap) {
    leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
    leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
    rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
    od = hardwareMap.get(GoBildaPinpointDriver.class, "od");

    leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    od.resetPosAndIMU();
    od.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    od.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    od.setOffsets(0, 0);
    rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  public void reset() {
    yawOffset = od.getHeading();
  }

  public void moveRobotFieldRelative(double forward, double fun, double turn) {
    od.update();

    turn *= 0.8;
    double botHeading = od.getHeading() - yawOffset;
    // Rotate the movement direction counter to the bot's rotation\\
    double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
    double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

    rotX = rotX * 1.1; // Counteract imperfect strafing

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
    double leftFrontPower = (rotY + rotX + turn) / denominator;
    double leftBackPower = (rotY - rotX + turn) / denominator;
    double rightFrontPower = (rotY - rotX - turn) / denominator;
    double rightBackPower = (rotY + rotX - turn) / denominator;

    leftFrontMotor.setPower(leftFrontPower);
    leftBackMotor.setPower(leftBackPower);
    rightFrontMotor.setPower(rightFrontPower);
    rightBackMotor.setPower(rightBackPower);
  }

  public void moveRobot(double forward, double fun, double turn) {
    double rotX = forward * 1.1; // Counteract imperfect strafing
    double rotY = fun;

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
    double leftFrontPower = (rotY + rotX + turn) / denominator;
    double leftBackPower = (rotY - rotX + turn) / denominator;
    double rightFrontPower = (rotY - rotX - turn) / denominator;
    double rightBackPower = (rotY + rotX - turn) / denominator;

    leftFrontMotor.setPower(leftFrontPower);
    leftBackMotor.setPower(leftBackPower);
    rightFrontMotor.setPower(rightFrontPower);
    rightBackMotor.setPower(rightBackPower);
  }

  public void stop() {
    moveRobot(0, 0, 0);
  }
}
