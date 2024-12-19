package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber extends SubsystemBase {
  private final DcMotorEx climber;
  private final Servo climberLockServo;
  private final Servo slideLockServo;
  private boolean isOpen = false;

  public Climber(final HardwareMap hardwareMap) {
    climber = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
    climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    climber.setDirection(DcMotorSimple.Direction.REVERSE);
    climberLockServo = hardwareMap.get(Servo.class, "climberLockServo");
    slideLockServo = hardwareMap.get(Servo.class, "slideLockServo");
    climberLockServo.setPosition(0);
    slideLockServo.setPosition(0);
  }

  public void switchClimberLock() {
    if (isOpen) {
      climberLockServo.setPosition(1);
    } else {
      climberLockServo.setPosition(0);
    }
    isOpen = !isOpen;
  }

  public void closeClimberLock() {
    climberLockServo.setPosition(0);
  }

  public void openClimberLock() {
    climberLockServo.setPosition(0);
  }

  public void closeSlideLock() {
    slideLockServo.setPosition(0);
  }

  public void openSlideLock() {
    slideLockServo.setPosition(0);
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
