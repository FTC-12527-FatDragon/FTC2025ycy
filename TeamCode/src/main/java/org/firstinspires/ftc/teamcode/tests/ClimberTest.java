package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Climber;

@TeleOp(name = "Test TeleOp")
public class ClimberTest extends CommandOpMode {
  private Climber climber;

  @Override
  public void initialize() {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    climber = new Climber(hardwareMap);
    climber.elevate();
    telemetry.addLine("Elevator finished!");
  }
}
