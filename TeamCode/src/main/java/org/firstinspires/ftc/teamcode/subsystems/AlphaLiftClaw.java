package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;
import static org.firstinspires.ftc.teamcode.utils.ServoUtils.setServoPosCommand;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;

public class AlphaLiftClaw extends SubsystemBase {
  private final Servo liftArmServo;
  private final Servo liftClawServo;
  private final Servo liftWristServo;
  private final Telemetry telemetry;

  public AlphaLiftClaw(final HardwareMap hardwareMap, Telemetry telemetry) {
    liftArmServo = hardwareMap.get(Servo.class, "liftArmServo"); // 0.3 Up 0.7 Down
    liftClawServo = hardwareMap.get(Servo.class, "liftClawServo"); // 0 Close 0.5 Open
    liftWristServo = hardwareMap.get(Servo.class, "liftWristServo");
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  }

  public void initialize() {
    liftWristServo.setPosition(ServoPositions.STOW.liftWristPosition);
    liftArmServo.setPosition(ServoPositions.STOW.liftArmPosition);
    liftClawServo.setPosition(ServoPositions.STOW.liftClawPosition);
  }

  public void autoInitialize() {
    liftWristServo.setPosition(ServoPositions.STOW.liftWristPosition);
    liftArmServo.setPosition(ServoPositions.STOW.liftArmPosition);
    liftClawServo.setPosition(ServoPositions.STOW.liftClawPosition);
  }

  public void grabWrist() {
    liftWristServo.setPosition(ServoPositions.GRAB.liftWristPosition);
  }

  public void basketWrist() {
    liftWristServo.setPosition(ServoPositions.BASKET.liftWristPosition);
  }

  public void stowWrist() {
    liftWristServo.setPosition(ServoPositions.STOW.liftWristPosition);
  }

  public void chamberWrist() {
    liftWristServo.setPosition(ServoPositions.CHAMBER.liftWristPosition);
  }

  public Command switchLiftClawCommand() {
    if (liftClawServo.getPosition() != ServoPositions.GRAB.liftClawPosition) {
      return new InstantCommand(this::openClaw);
    } else {
      return closeClawCommand();
    }
  }

  public void openClaw() {
    liftClawServo.setPosition(ServoPositions.GRAB.liftClawPosition);
  }

  public Command closeClawCommand(long delay) {
    return setServoPosCommand(liftClawServo, ServoPositions.STOW.liftClawPosition, delay);
  }

  public Command closeClawCommand() {
    return closeClawCommand(200);
  }

  public void upLiftArm() {
    liftArmServo.setPosition(ServoPositions.BASKET.liftArmPosition);
  }

  public Command foldLiftArmCommand(long delay) {
    return setServoPosCommand(liftArmServo, ServoPositions.STOW.liftArmPosition, delay);
  }

  public Command foldLiftArmCommand() {
    return foldLiftArmCommand(200);
  }

  public void grabLiftArm() {
    liftArmServo.setPosition(ServoPositions.GRAB.liftArmPosition);
  }

  public void chamberLiftArm() {
    liftArmServo.setPosition(ServoPositions.CHAMBER.liftArmPosition);
  }

  public enum ServoPositions {
    STOW(0.83, currentRobot==DriveConstants.RobotType.ALPHA?0.33:0.35, currentRobot==DriveConstants.RobotType.ALPHA?0.63:0.595),
    CHAMBER(0.66, currentRobot==DriveConstants.RobotType.ALPHA?0.33:0.35, currentRobot==DriveConstants.RobotType.ALPHA?0.27:0.47),
    BASKET(currentRobot==DriveConstants.RobotType.ALPHA?0.45:0.46, currentRobot==DriveConstants.RobotType.ALPHA?0.33:0.35, 0.5),
    GRAB(currentRobot== DriveConstants.RobotType.ALPHA?0.23:0.225, currentRobot==DriveConstants.RobotType.ALPHA?0.7:0.7025, currentRobot==DriveConstants.RobotType.ALPHA?0.08:0.39);

    private final double liftArmPosition;
    private final double liftWristPosition;
    private final double liftClawPosition;

    ServoPositions(double liftArmPosition, double liftClawPosition, double liftWristPosition) {
      this.liftArmPosition = liftArmPosition;
      this.liftClawPosition = liftClawPosition;
      this.liftWristPosition = liftWristPosition;
    }
  }

  @Override
  public void periodic() {
    telemetry.addData("Lift Arm Position", liftArmServo.getPosition());
    telemetry.update();
  }
}
