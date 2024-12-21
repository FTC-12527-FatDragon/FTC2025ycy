package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Climber extends SubsystemBase {
  public static long climberUpMs = 2000;
  private final DcMotorEx climber;
  private final Servo climberLockServo;
  private final Servo slideLockServo;
  private static final double climberLockClose = 0;
  private static final double climberLockOpen = 0.3;
  private static final double slideLockClose = 0.69;
  private static final double slideLockOpen = 0.1;
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
      climberLockServo.setPosition(climberLockClose);
    } else {
      climberLockServo.setPosition(climberLockOpen);
    }
    isOpen = !isOpen;
  }

  public void closeClimberLock() {
    climberLockServo.setPosition(climberLockClose);
  }

  public void openClimberLock() {
    climberLockServo.setPosition(climberLockOpen);
  }

  public void closeSlideLock() {
    slideLockServo.setPosition(slideLockClose);
  }

  public void openSlideLock() {
    slideLockServo.setPosition(slideLockOpen);
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

  public void keep() {
    climber.setPower(-0.2);
  }

  public Command elevateCommand() {
    return new StartEndCommand(this::elevate, this::stop);
  }

  public Command declineCommand() {
    return new StartEndCommand(this::decline, this::keep);
  }

  public Command decline2ArmUp() {
    return new WaitCommand(climberUpMs).deadlineWith(declineCommand());
  }

  public Command elevate2ArmDown() {
    return new WaitCommand(climberUpMs).deadlineWith(elevateCommand());
  }
}
