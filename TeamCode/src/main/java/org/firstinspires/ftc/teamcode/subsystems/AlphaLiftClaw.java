package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AlphaLiftClaw extends SubsystemBase {
  private final Servo liftArmServo;
  private final Servo liftClawServo;
  private final Servo liftWristServo;
  private boolean isClawOpen;
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

  public void grabWrist() {
    liftWristServo.setPosition(ServoPositions.GRAB.liftWristPosition);
  }

  public void basketWrist() {
    liftWristServo.setPosition(ServoPositions.BASKET.liftWristPosition);
  }

  public void chamberWrist() {
    liftWristServo.setPosition(ServoPositions.CHAMBER.liftWristPosition);
  }

  public void switchLiftClaw() {
    if (isClawOpen) {
      openClaw();
    } else {
      closeClaw();
    }
    isClawOpen = !isClawOpen;
  }

  public void openClaw() {
    liftClawServo.setPosition(ServoPositions.GRAB.liftClawPosition);
  }

  public void closeClaw() {
    liftClawServo.setPosition(ServoPositions.STOW.liftClawPosition);
  }

  public void upLiftArm() {
    liftArmServo.setPosition(ServoPositions.BASKET.liftArmPosition);
  }

  public void foldLiftArm() {
    liftArmServo.setPosition(ServoPositions.STOW.liftArmPosition);
  }

  public void grabLiftArm() {
    liftArmServo.setPosition(ServoPositions.GRAB.liftArmPosition);
  }

  public void chamberLiftArm() {
    liftArmServo.setPosition(ServoPositions.CHAMBER.liftArmPosition);
  }

  public enum ServoPositions {
    STOW(0.93, 0.27, 0.65),
    CHAMBER(0.68, 0.27, 0.26),
    BASKET(0.39, 0.27, 0.56),
    GRAB(0.08, 0.5, 0.06);

    private double liftArmPosition;
    private double liftWristPosition;
    private double liftClawPosition;

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


