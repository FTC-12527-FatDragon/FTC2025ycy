package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.EmptyMotor;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.ParallelRaceGroup;

@Config
public class Lift extends MotorPIDSlideSubsystem {
  public static double kP = 0.008, kI = 0.0, kD = 0.0, kV = 0.0003, kS = 0.12, kG = 0.12;
  private final PIDController pidController;
  private final Motor liftMotorUp;
  private final Motor liftMotorDown;

  private double lastSetpoint = 0;

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private final ElapsedTime timer;
  private double lastTime;

  //  private boolean isResetting = false;
  public static double resetPower = -0.7;
  public static double hangAddtionalPower = 0;

  public static double reasonableUp1000TicksMaxTimeMs = 1500;

  public static double MAX_VEL = 0;
  public static double MAX_ACL = 0;

  private final ElevatorFeedforward feedforward;

  private final VoltageSensor batteryVoltageSensor;

  @Getter private Goal goal = Goal.STOW;

  public static double AtGoalTolerance = 10;

  public Lift(final HardwareMap hardwareMap, Telemetry telemetry) {
    if (DriveConstants.currentRobot== DriveConstants.RobotType.ALPHA) {
      liftMotorUp = new Motor(hardwareMap, "liftMotor");
      liftMotorDown = new EmptyMotor();
      liftMotorUp.setInverted(true);
    } else {
      liftMotorUp = new Motor(hardwareMap, "liftMotorUp");
      liftMotorDown = new Motor(hardwareMap, "liftMotorDown");
      liftMotorUp.setInverted(false);
    }



    liftMotorUp.stopAndResetEncoder();
    liftMotorDown.stopAndResetEncoder();
    liftMotorUp.setRunMode(Motor.RunMode.RawPower);
    liftMotorDown.setRunMode(Motor.RunMode.RawPower);

    pidController = new PIDController(kP, kI, kD);
    pidController.setIntegrationBounds(-1 / kI, 1 / kI);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    this.telemetry = telemetry;

    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACL));
    timer = new ElapsedTime();
    timer.reset();
    lastTime = timer.time(TimeUnit.MILLISECONDS);
  }

  public void runOpenLoop(double percent) {
    //    goal = Goal.OPEN_LOOP;
//    if (percent == 0) {
//      isResetting = false;
//    } else {
//      isResetting = true;
//    }
    double output = Range.clip(percent, -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);
  }

  public double getResetPower() {
    return resetPower;
  }

  public void resetEncoder() {
    runOpenLoop(0);
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

  public long getCurrentPosition() {
    return liftMotorUp.getCurrentPosition();
  }

  public boolean atGoal() {
    return MathUtils.isNear(goal.setpointTicks, getCurrentPosition(), AtGoalTolerance);
  }

  public boolean atHome(double tolerance) {
    return MathUtils.isNear(Goal.STOW.setpointTicks, getCurrentPosition(), tolerance);
  }

  public Command waitAtGoal(){
    return new WaitUntilCommand(this::atGoal);
  }

  public Command waitAtGoal(long timeoutMs, Runnable onTimeout){
    return new ParallelRaceGroup(
            new WaitCommand(timeoutMs).andThen(new InstantCommand(onTimeout)),
            waitAtGoal()
    );
  }

  public Command setGoalCommand(Goal newGoal, boolean wait){
    double dl = Math.abs(getCurrentPosition() - newGoal.setpointTicks);
    Command toRun = new InstantCommand(() -> goal = newGoal);
    if(wait){
      return toRun.andThen(waitAtGoal((long)Math.ceil(reasonableUp1000TicksMaxTimeMs*dl*0.001), () ->
              telemetry.log().add("Function Lift::waitAtGoal exceeded the reasonable time, please check hardware status")
      ));
    }else{
      return toRun;
    }
  }

  public Command setGoalCommand(Goal newGoal){
    return setGoalCommand(newGoal, true);
  }

  public boolean atPreHang() {
    return MathUtils.isNear(Goal.PRE_HANG.setpointTicks, getCurrentPosition(), AtGoalTolerance);
  }

  public void periodicTest() {
    telemetry.addData("Lift.Current Goal", goal.toString());
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

    double pidPower = pidController.calculate(getCurrentPosition(), setpointState.position);
    double output = pidPower + feedforward.calculate(setpointState.velocity);
    if (goal == Goal.HANG) {
      output += hangAddtionalPower;
    }
    output *= 12 / batteryVoltageSensor.getVoltage();
    output = Range.clip(output, -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);

    lastTime = timer.time(TimeUnit.MILLISECONDS);

    telemetry.addData("Lift.PID Power", pidPower);
    telemetry.addData("Lift.output", output);

    // telemetry.update();
  }

  public enum Goal {
    BASKET(1500),
    STOW(0),
    PRE_HANG(600.0),
    HANG(1010.0),
    GRAB(0),
    OPEN_LOOP(0.0);

    private final double setpointTicks;

    Goal(double setpointTicks) {
      this.setpointTicks = setpointTicks;
    }

    @NonNull
    public String toString(){
      return "Goal."+name()+"("+setpointTicks+")";
    }
  }
}
