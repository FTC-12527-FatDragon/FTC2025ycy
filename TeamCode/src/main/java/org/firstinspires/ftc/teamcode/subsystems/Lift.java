package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.concurrent.TimeUnit;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class Lift extends SubsystemBase {
  private final double kP = 0.01, kI = 0.0, kD = 0.0, kV = 0.0003, kS = 0.12, kG = 0.15;
  private final PIDController pidController;
  private final Motor liftMotorUp;
  private final Motor liftMotorDown;
  private final Telemetry telemetry;

  private double lastSetpoint = 0;

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private final ElapsedTime timer;
  private double lastTime;

  private final ElevatorFeedforward feedforward;

  @Getter @Setter private Goal goal = Goal.STOW;

  public Lift(final HardwareMap hardwareMap, Telemetry telemetry) {
    liftMotorUp = new Motor(hardwareMap, "liftMotorUp");
    liftMotorDown = new Motor(hardwareMap, "liftMotorDown");
    liftMotorUp.stopAndResetEncoder();
    liftMotorDown.stopAndResetEncoder();
    liftMotorUp.setRunMode(Motor.RunMode.RawPower);
    liftMotorDown.setRunMode(Motor.RunMode.RawPower);

    pidController = new PIDController(kP, kI, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
    this.telemetry = telemetry;

    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(15000, 15000));
    timer = new ElapsedTime();
    timer.reset();
    lastTime = timer.time(TimeUnit.MILLISECONDS);
  }

  public void runLiftOpen(double percent) {
    goal = Goal.OPEN_LOOP;
    double output = Range.clip(percent, -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);
  }

  public Command resetCommand() {
    return new StartEndCommand(
        () -> {
          runLiftOpen(-0.3);
        },
        () -> {
          pidController.reset();
          pidController.calculate(0);
          runLiftOpen(0);
          goal = Goal.STOW;
          telemetry.addData("Lift Current Position", liftMotorUp.getCurrentPosition());
          telemetry.addData("Error", pidController.getPositionError());
          // telemetry.update();
        },
        this);
  }

  public double getCurrentPosition() {
    return liftMotorUp.getCurrentPosition();
  }

  public boolean atGoal() {
    return MathUtils.isNear(goal.setpointTicks, liftMotorUp.getCurrentPosition(), 10);
  }

  public boolean atHome() {
    return MathUtils.isNear(Goal.STOW.setpointTicks, liftMotorUp.getCurrentPosition(), 10);
  }

  public boolean atPreHang() {
    return MathUtils.isNear(Goal.PRE_HANG.setpointTicks, liftMotorUp.getCurrentPosition(), 10);
  }

  public void periodicTest() {
    if (goal == Goal.OPEN_LOOP) return;

    if (lastSetpoint != goal.setpointTicks) {
      goalState = new TrapezoidProfile.State(goal.setpointTicks, 0);
      lastSetpoint = goal.setpointTicks;
    }

    double timeInterval =
        Range.clip((timer.time(TimeUnit.MILLISECONDS) - lastTime) * 0.001, 0.001, 0.05);

    telemetry.addData("Time Interval", timeInterval);
    setpointState = profile.calculate(timeInterval, setpointState, goalState);

    double pidPower =
        pidController.calculate(liftMotorUp.getCurrentPosition(), setpointState.position);
    double output = Range.clip(pidPower + feedforward.calculate(setpointState.velocity), -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);

    lastTime = timer.time(TimeUnit.MILLISECONDS);

    telemetry.addData("Current Goal", goal);
    telemetry.addData("At Goal", atGoal());
    telemetry.addData("Current Position", liftMotorUp.getCurrentPosition());
    telemetry.addData("PID Power", pidPower);
    // telemetry.update();
  }

  public enum Goal {
    BASKET(760.0),
    STOW(0.0),
    PRE_HANG(150.0),
    HANG(0),
    OPEN_LOOP(0.0);

    private final double setpointTicks;

    Goal(double setpointTicks) {
      this.setpointTicks = setpointTicks;
    }
  }
}
