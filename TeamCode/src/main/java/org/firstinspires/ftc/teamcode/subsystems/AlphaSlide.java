package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AlphaSlide extends SubsystemBase {
  private final Servo intakeClawServo, wristServo, wristTurnServo;
  private final Servo slideArmServo, slideRightServo;
  private boolean hasGamepiece = false;
  private static double slideExtensionVal = 0.17;

  private static double turnAngleDeg = 0;
  private TurnServo turnServo = TurnServo.DEG_0;

  @Setter @Getter private Goal goal = Goal.STOW;
  private boolean isIntakeClawOpen = false;

  private final Telemetry telemetry; // 0 0.5 0.8

  @Getter @Setter private boolean normalHandoff = false;

  public AlphaSlide(final HardwareMap hardwareMap, final Telemetry telemetry) {
    slideArmServo = hardwareMap.get(Servo.class, "slideArmServo"); // 0.5 up 0.9 half 1 down

    slideRightServo = hardwareMap.get(Servo.class, "slideLeftServo"); // 1 stow

    intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo"); // 0.3 close 0.7 open
    wristServo = hardwareMap.get(Servo.class, "wristServo"); // 0.05 up 0.75 down

    wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");
    this.telemetry = new MultipleTelemetry();
    goal = Goal.STOW;
    telemetry.addData("Current State", goal);
    // telemetry.update();

  }

  public Command aimCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal = Goal.AIM),
        new InstantCommand(
            () -> {
              handoffWristTurn();
              turnServo = TurnServo.DEG_0;
            }),
        new WaitCommand(100),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.AIM.slideArmPos)),
        new WaitCommand(100),
        new InstantCommand(() -> wristServo.setPosition(Goal.AIM.wristPos)),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle)));
  }

  public Command grabCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal = Goal.GRAB),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.GRAB.slideArmPos)),
        new WaitCommand(100),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.GRAB.clawAngle)),
        new WaitCommand(100),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
        new InstantCommand(() -> goal = Goal.AIM));
  }

  public Command handoffCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal = Goal.HANDOFF),
        new InstantCommand(
            () -> {
              handoffWristTurn();
              turnServo = TurnServo.DEG_0;
            }),
        new WaitCommand(100),
        new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos)),
        new WaitCommand(200),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
        new WaitCommand(300),
        new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension));
  }

  public void initialize() {
    slideArmServo.setPosition(Goal.HANDOFF.slideArmPos);
    slideRightServo.setPosition(Goal.HANDOFF.slideExtension);
    intakeClawServo.setPosition(Goal.HANDOFF.clawAngle);
    wristServo.setPosition(Goal.HANDOFF.wristPos);
    wristTurnServo.setPosition(Goal.HANDOFF.turnAngle);
    handoffWristTurn();
    turnServo = TurnServo.DEG_0;
  }

  public void handoffWristTurn() {
    turnAngleDeg = TurnServo.DEG_0.turnPosition;
  }

  public void openIntakeClaw() {
    intakeClawServo.setPosition(Goal.STOW.clawAngle);
    isIntakeClawOpen = true;
  }

  public void closeIntakeClaw() {
    intakeClawServo.setPosition(Goal.HANDOFF.clawAngle);
    isIntakeClawOpen = false;
  }

  public void wristUp() {
    wristServo.setPosition(0.75);
  }

  public void wristDown() {
    wristServo.setPosition(0.05);
  }

  public void slideArmDown() {
    // This is down for stowing the liftArm when scoring the speciemen
    slideArmServo.setPosition(0.31);
  }

  public void slideArmUp() {
    // This is up for the auto
    slideArmServo.setPosition(0.6);
  }

  public enum Goal {
    STOW(0.1, 0.6, 0.31, 0.4, 0.5),
    AIM(slideExtensionVal, 0.45, 0.80, turnAngleDeg, 0.5),
    GRAB(slideExtensionVal, 0.31, 0.80, turnAngleDeg, 0.23),
    HANDOFF(0.17, 0.6, 0.31, 0.4, 0.23);

    private final double slideExtension;
    private final double slideArmPos;
    private final double wristPos;
    private final double turnAngle;
    private final double clawAngle;

    Goal(
        double slideExtension,
        double slideArmPos,
        double wristPos,
        double turnAngle,
        double clawAngle) {
      this.slideExtension = slideExtension;
      this.slideArmPos = slideArmPos;
      this.wristPos = wristPos;
      this.turnAngle = turnAngle;
      this.clawAngle = clawAngle;
    }
  }

  public void forwardSlideExtension() {
    slideExtensionVal = 0.76;
  }

  public void backwardSlideExtension() {
    slideExtensionVal = 0.17;
  }

  private final double isSlideExtendedVal = 0.3;

  public void preHandoffSlideExtension() {
    slideExtensionVal = isSlideExtendedVal;
  }

  public boolean isSlideForward() {
    return slideExtensionVal > isSlideExtendedVal;
  }

  public void leftTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnAngleDeg = TurnServo.DEG_0.turnPosition;
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_45:
        turnAngleDeg = TurnServo.DEG_0.turnPosition;
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_90:
        turnAngleDeg = TurnServo.DEG_45.turnPosition;
        turnServo = TurnServo.DEG_45;
        break;
    }
  }

  public void rightTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnAngleDeg = TurnServo.DEG_45.turnPosition;
        turnServo = TurnServo.DEG_45;
        break;
      case DEG_45:
        turnAngleDeg = TurnServo.DEG_90.turnPosition;
        turnServo = TurnServo.DEG_90;
        break;
      case DEG_90:
        turnAngleDeg = TurnServo.DEG_90.turnPosition;
        turnServo = TurnServo.DEG_90;
        break;
    }
  }

  enum TurnServo {
    DEG_0(0.4),
    DEG_45(0.55),
    DEG_90(0.7);

    private double turnPosition;

    TurnServo(double turnPosition) {
      this.turnPosition = turnPosition;
    }
  }

  @Override
  public void periodic() {

    if (wristTurnServo != null) {
      wristTurnServo.setPosition(Range.clip(turnAngleDeg, 0, 1));
      slideRightServo.setPosition(Range.clip(slideExtensionVal, 0, 1));

      telemetry.addData("Current State", goal);
      telemetry.addData("Bur Gemen", goal == Goal.HANDOFF);
      telemetry.addData("Claw Position", intakeClawServo.getPosition());
      telemetry.addData("Slide Extension", slideExtensionVal);
      telemetry.addData("Turn Angle", turnAngleDeg);
      telemetry.addData("SLideServo Position", slideRightServo.getPosition());
    }
    // slidetelemetry.update();
  }
}

/*
@Config
public class AlphaSlide extends SubsystemBase {
    public static double slideServo_ExtendPose = 0.73;
    public static double slideServo_ContractPose = 0.17;
    public static double clawServo_OpenPose = 0.4;
    public static double clawServo_GrabPose = 0.66;
    public static double clawTurnServo_MinPose = 0;
    public static double clawTurnServo_MaxPose = 1;
    private final HardwareMap hardwareMap;
    private final Servo clawServo, clawTurnServo, wristServo, turnServo, slideServo;
    private double clawTurnServoSetpoint = clawTurnServo_MinPose;
    public AlphaSlide(HardwareMap hm){
        hardwareMap = hm;
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        clawTurnServo = hardwareMap.get(Servo.class,"clawTurnServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        turnServo = hardwareMap.get(Servo.class, "turnServo");
        slideServo = hardwareMap.get(Servo.class, "slideServo");
        slideServo.setDirection(Servo.Direction.FORWARD);
        clawTurnServo.setDirection(Servo.Direction.REVERSE);
        setDefaultYaw();
    }
    public void aim(Pose2d blockPose){
        // TODO: complete the code
    }

    public void aim(double slideSetpoint, double yawSetpointRot){
        // TODO: calc setpoint based on joint, to make it linear
        slideServo.setPosition(slideSetpoint*(slideServo_ExtendPose - slideServo_ContractPose)+slideServo_ExtendPose);
//        clawServo.setPosition(clawServo_OpenPose);
        setYaw(yawSetpointRot);
    }
    public void grab(){
        clawServo.setPosition(clawServo_GrabPose);
    }
    public void retract(){
        slideServo.setPosition(slideServo_ContractPose);
        setDefaultYaw();
    }
    public void release(){
        clawServo.setPosition(clawServo_OpenPose);
    }
    public void setYaw(double yawSetpointRot){
        yawSetpointRot *= 2;
        yawSetpointRot += 0.5;
        clawTurnServoSetpoint = yawSetpointRot - Math.floor(yawSetpointRot);
        updateClawTurnServo();
    }
    public void setDefaultYaw(){
        clawTurnServoSetpoint = (clawTurnServo_MaxPose + clawTurnServo_MinPose) / 2;
        updateClawTurnServo();
    }
    private void updateClawTurnServo(){
        clawTurnServo.setPosition(clawTurnServoSetpoint);
    }
    public Command aimCommand(Supplier<Pose2d> getBlockPos){
        return new InstantCommand(() -> aim(getBlockPos.get()));
    }
    public Command grabCommand(){
        return new InstantCommand(this::grab);
    }
    public Command retractCommand(){
        return new InstantCommand(this::retract);
    }

    public static enum SlidePosition {
        AIM(0,0,0,0,0),
        GRAB(0,0,0,0,0),
        HANDOFF(0,0,0,0,0);

        private double slidePosition, clawPosition, armPosition, turnPosition, wristPosition;

        SlidePosition(double slidePosition, double clawPosition,
                      double armPosition, double turnPosition, double wristPosition){
            this.slidePosition = slidePosition;
            this.clawPosition = clawPosition;
            this.armPosition = armPosition;
            this.turnPosition = turnPosition;
            this.wristPosition = wristPosition;

        }
    }
}*/
