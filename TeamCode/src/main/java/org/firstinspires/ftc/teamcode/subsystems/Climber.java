package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber extends SubsystemBase {
  private final DcMotorEx climber;
  private final Servo lockServo;
  private boolean isOpen = false;

  public Climber(final HardwareMap hardwareMap) {
    climber = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
    climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lockServo = hardwareMap.get(Servo.class, "lockServo");
    lockServo.setPosition(0);
  }

  public void switchLock() {
    if (isOpen) {
      lockServo.setPosition(1);
    } else {
      lockServo.setPosition(0);
    }
    isOpen = !isOpen;
  }

  public void elevate() {
    climber.setPower(1);
  }

  public void decline() {
    climber.setPower(-1);
  }

  public void stop() {
    climber.setPower(0);
  }

  public Command elevateCommand() {
    return new StartEndCommand(this::elevate, this::stop);
  }

  public Command declineCommand() {
    return new StartEndCommand(this::decline, this::stop);
  }
}
