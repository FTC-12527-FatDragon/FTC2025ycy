package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.concurrent.TimeUnit;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

@Config
public class Lift extends SubsystemBase {
  public static double kP = 0.01, kI = 0.001, kD = 0.0, kV = 0.0003, kS = 0.12, kG = 0.12;
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

  private boolean isResetting = false;
  public static double resetPower = -0.5;

  private final ElevatorFeedforward feedforward;

  private final VoltageSensor batteryVoltageSensor;

  @Getter @Setter private Goal goal = Goal.STOW;

  public Lift(final HardwareMap hardwareMap, Telemetry telemetry) {
    liftMotorUp = new Motor(hardwareMap, "liftMotorUp");
    liftMotorDown = new Motor(hardwareMap, "liftMotorDown");
    liftMotorUp.stopAndResetEncoder();
    liftMotorDown.stopAndResetEncoder();
    liftMotorUp.setRunMode(Motor.RunMode.RawPower);
    liftMotorDown.setRunMode(Motor.RunMode.RawPower);

    pidController = new PIDController(kP, kI, kD);
    pidController.setIntegrationBounds(-1/kI, 1/kI);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    this.telemetry = telemetry;

    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(15000, 15000));
    timer = new ElapsedTime();
    timer.reset();
    lastTime = timer.time(TimeUnit.MILLISECONDS);
  }

  public void runLiftOpen(double percent) {
//    goal = Goal.OPEN_LOOP;
    if(percent==0){
      isResetting = false;
    }else{
      isResetting = true;
    }
    double output = Range.clip(percent, -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);
  }

  public void resetEncoder() {
    runLiftOpen(0);
    pidController.reset();
    pidController.calculate(0);
    goal = Goal.STOW;
    // TODO: does this work?
    liftMotorUp.resetEncoder();
    liftMotorDown.resetEncoder();
    telemetry.addData("Lift.Current Position", getCurrentPosition());
    telemetry.addData("Lift.Error", pidController.getPositionError());
    // telemetry.update();
  }

  public Command manualResetCommand() {
    return new StartEndCommand(
        () -> {
          runLiftOpen(resetPower);
        },
        this::resetEncoder,
        this);
  }

  public Command autoResetCommand(){
    return new CommandBase() {
      double minVal = Double.POSITIVE_INFINITY;
      double velo, lastvelo;
      double startpos, lastpos, lastTime = 0;
      final ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
      @Override
      public void initialize () {
        lastpos = getCurrentPosition();
        startpos = lastpos;
        isResetting = true;
        velo = 0;
        t.reset();
        runLiftOpen(resetPower);
      }
      @Override
      public void execute () {
        lastvelo = velo;
        double nowpos = getCurrentPosition();
        double nowtime = t.time();
        velo = (nowpos - lastpos) / (nowtime - lastTime);
        lastTime = nowtime;
        if (minVal > nowpos) {
          minVal = nowpos;
        }
        lastpos = nowpos;
        telemetry.addData("Lift.AutoResetCommand.vel", velo);
        telemetry.addData("Lift.AutoResetCommand.dt", lastpos - nowtime);
        telemetry.addData("Lift.AutoResetCommand.minVal", minVal);
        telemetry.addData("Lift.AutoResetCommand.AvgVel", ((getCurrentPosition() - startpos) / t.time()));
      }
      @Override
      public boolean isFinished() {
        if(Math.abs(velo)<Math.abs(((getCurrentPosition() - startpos) / t.time())*0.9) && lastvelo>velo){
          return true;
        }else return false;
      }
      @Override
      public void end(boolean interrupted) {
        runLiftOpen(0);
        if(!interrupted){
          resetEncoder();
        }
        isResetting = false;
      }
    };
  }

  public long getCurrentPosition() {
    return liftMotorUp.getCurrentPosition();
  }

  public boolean atGoal() {
    return MathUtils.isNear(goal.setpointTicks, getCurrentPosition(), 8 );
  }

  public boolean atHome(double tolerance) {
    return MathUtils.isNear(Goal.STOW.setpointTicks, getCurrentPosition(), tolerance);
  }

  public boolean atPreHang() {
    return MathUtils.isNear(Goal.PRE_HANG.setpointTicks, getCurrentPosition(), 10);
  }

  public void periodicTest() {
    telemetry.addData("Lift.Current Goal", goal);
    telemetry.addData("Lift.At Goal", atGoal());
    telemetry.addData("Lift.Current Position", getCurrentPosition());
    telemetry.addData("Lift.Is Resetting", isResetting);
    if (goal == Goal.OPEN_LOOP || isResetting) return;

    if (lastSetpoint != goal.setpointTicks) {
      goalState = new TrapezoidProfile.State(goal.setpointTicks, 0);
      lastSetpoint = goal.setpointTicks;
    }

    double timeInterval =
        Range.clip((timer.time(TimeUnit.MILLISECONDS) - lastTime) * 0.001, 0.001, 0.05);

    telemetry.addData("Time Interval", timeInterval);
    setpointState = profile.calculate(timeInterval, setpointState, goalState);

    double pidPower =
            pidController.calculate(getCurrentPosition(), setpointState.position);
    double output = pidPower + feedforward.calculate(setpointState.velocity);
    if(goal==Goal.HANG){
      output -= 1;
    }
    output *= 12/batteryVoltageSensor.getVoltage();
    output = Range.clip(output, -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);

    lastTime = timer.time(TimeUnit.MILLISECONDS);

    telemetry.addData("Lift.PID Power", pidPower);
    telemetry.addData("Lift.output", output);

    // telemetry.update();
  }

  public enum Goal {
    BASKET(760.0),
    STOW(0.0),
    PRE_HANG(144.0),
    HANG(0),
    OPEN_LOOP(0.0);

    private final double setpointTicks;

    Goal(double setpointTicks) {
      this.setpointTicks = setpointTicks;
    }
  }
}
