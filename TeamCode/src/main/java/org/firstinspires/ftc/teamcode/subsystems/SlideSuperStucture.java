package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class SlideSuperStucture extends MotorPIDSlideSubsystem {
  private final Servo intakeClawServo, wristServo, wristTurnServo;
  private final Servo slideArmServo;
  private final DcMotorEx slideMotor;

  private final PIDController pidController;
  private final double kP = 0.04, kI = 0.0, kD = 0.0;
  private final VoltageSensor batteryVoltageSensor;

  private boolean hasGamepiece = false;
  private static double slideExtensionVal = 0;

  private static double turnAngleDeg = 0.2;
  private TurnServo turnServo = TurnServo.DEG_0;

  @Setter @Getter private Goal goal = Goal.STOW;

  //  private final Telemetry telemetry; // 0 0.5 0.8

  //  private boolean isResetting = false;
  public static double resetPower = -0.5;

  public SlideSuperStucture(final HardwareMap hardwareMap, final Telemetry telemetry) {
    slideArmServo = hardwareMap.get(Servo.class, "slideArmServo");

    intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo"); // 0.3 close 0.7 open

    wristServo = hardwareMap.get(Servo.class, "wristServo"); // 0.05 up 0.75 down

    wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");

    setServoController(true);

    slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
    slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    pidController = new PIDController(kP, kI, kD);
    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    this.telemetry = telemetry;
    goal = Goal.STOW;
  }

  public Command setGoalCommand(Goal newGoal) {
    return new InstantCommand(() -> goal = newGoal);
  }

  public Command aimCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.AIM),
        new InstantCommand(
            () -> {
              setServoPos(TurnServo.DEG_0);
            }),
        new WaitCommand(100),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.AIM.slideArmPos)),
        new WaitCommand(100),
        new InstantCommand(() -> wristServo.setPosition(Goal.AIM.wristPos)),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle)));
  }

  public Command grabCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.GRAB),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.GRAB.slideArmPos)),
        new WaitCommand(100),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.GRAB.clawAngle)),
        new WaitCommand(100),
        new InstantCommand(() -> slideArmServo.setPosition(0.5)),
        setGoalCommand(Goal.AIM));
  }

  public Command slowHandoffCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.HANDOFF),
        new InstantCommand(
            () -> {
              setServoPos(TurnServo.DEG_0);
            }),
        new WaitCommand(100),
        new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos)),
        new WaitCommand(300),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
        new WaitCommand(200),
        new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension),
        new WaitUntilCommand(this::slideMotorAtHome));
  }

  public Command fastHandoffCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.HANDOFF),
        new InstantCommand(
            () -> {
              setServoPos(TurnServo.DEG_0);
            }),
        new WaitCommand(100),
        new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos)),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
        new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension),
        new WaitUntilCommand(this::slideMotorAtHome));
  }

  public Command handoffCommand() {
    return new ConditionalCommand(slowHandoffCommand(), fastHandoffCommand(), this::atHome);
  }

  public void openIntakeClaw() {
    intakeClawServo.setPosition(Goal.AIM.clawAngle);
  }

  public void closeIntakeClaw() {
    intakeClawServo.setPosition(Goal.GRAB.clawAngle);
  }

  /** Up to avoid the collision with the clip */
  public void wristUp() {
    wristServo.setPosition(0.65);
  }

  public void wristDown() {
    wristServo.setPosition(0.05);
  }

  public void slideArmDown() {
    // This is down for stowing the liftArm when scoring the speciemen
    slideArmServo.setPosition(Goal.AIM.slideArmPos);
  }

  public void slideArmUp() {
    // This is up for the auto
    slideArmServo.setPosition(0.35);
  }

  public void stow() {
    slideArmServo.setPosition(Goal.HANDOFF.slideArmPos);
    wristServo.setPosition(Goal.HANDOFF.wristPos);
  }

  public enum Goal {
    STOW(0, 0, 0.2, 0.6),
    AIM(slideExtensionVal, 0.625, 0.65, 0.6),
    GRAB(slideExtensionVal, 0.725, 0.65, 0.36),
    HANDOFF(0, 0.45, 0.175, 0.36);

    public final double slideExtension;
    public final double slideArmPos;
    public final double wristPos;
    public final double clawAngle;

    Goal(double slideExtension, double slideArmPos, double wristPos, double clawAngle) {
      this.slideExtension = slideExtension;
      this.slideArmPos = slideArmPos;
      this.wristPos = wristPos;
      this.clawAngle = clawAngle;
    }
  }

  public void forwardSlideExtension() {
    forwardSlideExtension(460);
  }

  public void forwardSlideExtension(double slideExtension) {
    slideExtensionVal = slideExtension;
  }

  public void backwardSlideExtension() {
    slideExtensionVal = 0;
  }

  public void leftTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnServo = TurnServo.DEG_05;
        break;
      case DEG_05:
        turnServo = TurnServo.DEG_08;
        break;
      case DEG_08:
        turnServo = TurnServo.DEG_08;
        break;
    }
    setServoPos(turnServo);
  }

  public void rightTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_05:
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_08:
        turnServo = TurnServo.DEG_05;
        break;
    }
    setServoPos(turnServo);
  }

  public void setServoPos(TurnServo pos) {
    switch (pos) {
      case DEG_0:
        turnAngleDeg = 0.2;
        break;
      case DEG_05:
        turnAngleDeg = 0.4;
        break;
      case DEG_08:
        turnAngleDeg = 0.7;
        break;
    }
    turnServo = pos;
  }

  public Command setServoPosCommand(TurnServo pos) {
    return new InstantCommand(() -> setServoPos(pos));
  }

  public enum TurnServo {
    DEG_0,
    DEG_05,
    DEG_08
  }

  private boolean slideMotorAtGoal() {
    return MathUtils.isNear(goal.slideExtension, getCurrentPosition(), 10);
  }

  private boolean slideMotorAtHome() {
    return MathUtils.isNear(0, getCurrentPosition(), 10);
  }

  public long getCurrentPosition() {
    return slideMotor.getCurrentPosition();
  }

  public double getResetPower() {
    return resetPower;
  }

  @Override
  public void runOpenLoop(double percent) {
    double output = Range.clip(percent, -1, 1);
    slideMotor.setPower(output);
  }

  public void resetEncoder() {
    runOpenLoop(0);
    pidController.reset();
    pidController.calculate(0);
    // TODO: does this work?
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  //  public Command resetCommand() {
  //    return new StartEndCommand(
  //            () -> {
  //              runOpenLoop();
  //              isResetting = true;
  //            },
  //            () -> {
  //
  //              isResetting = false;
  //            },
  //            this);
  //  }

  public boolean atHome() {
    return MathUtils.isNear(getCurrentPosition(), 0, 5);
  }

  @Override
  public void periodic() {
    wristTurnServo.setPosition(Range.clip(turnAngleDeg, 0, 1));

    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    double setpointTicks = slideExtensionVal;

    telemetry.addData("Current Goal", goal);
    telemetry.addData("Goal Extension", setpointTicks);
    telemetry.update();

    double pidPower = pidController.calculate(getCurrentPosition(), setpointTicks);
    pidPower *= 12 / batteryVoltageSensor.getVoltage();
    if (!isResetting) slideMotor.setPower(Range.clip(pidPower, -1, 1));
  }

  public void setServoController(boolean enable) {
    if (enable) {
      intakeClawServo.getController().pwmEnable();
      wristTurnServo.getController().pwmEnable();
      wristTurnServo.getController().pwmEnable();
      slideArmServo.getController().pwmEnable();
    } else {
      intakeClawServo.getController().pwmDisable();
      wristTurnServo.getController().pwmDisable();
      wristTurnServo.getController().pwmDisable();
      slideArmServo.getController().pwmDisable();
    }
  }
}
