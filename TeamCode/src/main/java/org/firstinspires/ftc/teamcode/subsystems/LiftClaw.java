package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LiftClaw extends SubsystemBase {
  public static double LiftArmServo_UP = 0.725;
  public static double LiftArmServo_FOLD = 0.15;
  public static double ClawServo_CLOSE = 0.25;
  public static double ClawServo_OPEN = 0.5;
  private final Servo liftArmServo;
  private final Servo liftClawServo;

  public LiftClaw(final HardwareMap hardwareMap) {
    liftArmServo = hardwareMap.get(Servo.class, "liftArmServo"); // 0.3 Up 0.7 Down
    liftClawServo = hardwareMap.get(Servo.class, "clawServo"); // 0 Close 0.5 Open
    setServoController(true);
  }

  public void openClaw() {
    liftClawServo.setPosition(ClawServo_OPEN);
  }

  public void closeClaw() {
    liftClawServo.setPosition(ClawServo_CLOSE);
  }

  public void upLiftArm() {
    liftArmServo.setPosition(LiftArmServo_UP);
  }

  public void foldLiftArm() {
    liftArmServo.setPosition(LiftArmServo_FOLD);
  }

  public void setServoController(boolean enable) {
    if (enable) {
      liftArmServo.getController().pwmEnable();
      liftClawServo.getController().pwmEnable();
    } else {
      liftArmServo.getController().pwmDisable();
      liftClawServo.getController().pwmDisable();
    }
  }
}
