package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@TeleOp(name = "Test TeleOp")
public class ElevatorTest extends CommandOpMode {
  private Elevator elevator;

  @Override
  public void initialize() {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    elevator = new Elevator(hardwareMap);
    elevator.elevate();
    telemetry.addLine("Elevator finished!");
  }
}
