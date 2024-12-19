package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Climber extends SubsystemBase {
//  private final DcMotorEx elevator;

  public Climber(HardwareMap hardwareMap) {
//    elevator = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
  }

  public void elevate() {
//    elevator.setPower(1);
  }

  public void decline() {
//    elevator.setPower(-1);
  }

  public void stop() {
//    elevator.setPower(0);
  }

  public Command elevateCommand() {
    return new StartEndCommand(this::elevate, this::stop);
  }

  public Command declineCommand() {
    return new StartEndCommand(this::decline, this::stop);
  }
}
