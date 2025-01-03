package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;
import static org.firstinspires.ftc.teamcode.utils.ServoUtils.setServoPosCommand;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.MotorServo;

@Config
public class AlphaSlide extends SubsystemBase {

  // aimCommand
  public static long aimCommand_wristTurn2ArmDelayMs = 0;
  public static long aimCommand_Arm2OpenDelayMs = 20;

  private final Servo intakeClawServo, wristServo, wristTurnServo;
  private final Servo slideArmServo, slideRightServo;
  private boolean hasGamepiece = false;
  private static double slideExtensionVal = 0.21;
  private SlideServo slideServo = SlideServo.BACK;

  private static double turnAngleDeg = 0;
  private TurnServo turnServo = TurnServo.DEG_0;

  public static long grabTimeout = 200;

  @Setter @Getter private Goal goal = Goal.STOW;
  private boolean isIntakeClawOpen = false;

  private final Telemetry telemetry; // 0 0.5 0.8

  @Getter @Setter private boolean normalHandoff = false;

  public AlphaSlide(final HardwareMap hardwareMap, final Telemetry telemetry) {
    slideArmServo = hardwareMap.get(Servo.class, "slideArmServo"); // 0.5 up 0.9 half 1 down

    if(currentRobot== DriveConstants.RobotType.ALPHA){
      slideRightServo = hardwareMap.get(Servo.class, "slideLeftServo");
    }else slideRightServo = new MotorServo(hardwareMap, "slideMotor");//

    intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo"); // 0.3 close 0.7 open
    wristServo = hardwareMap.get(Servo.class, "wristServo"); // 0.05 up 0.75 down

    wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");
    this.telemetry = new MultipleTelemetry();
    goal = Goal.STOW;
    telemetry.addData("Current State", goal);
    // telemetry.update();
  }

  public Command setGoalCommand(Goal newGoal) {
    return new InstantCommand(() -> goal = newGoal);
  }

  public Command aimCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.AIM),
        setTurnServoPosCommand(TurnServo.DEG_0, aimCommand_wristTurn2ArmDelayMs),
        setServoPosCommand(slideArmServo, Goal.AIM.slideArmPos, aimCommand_Arm2OpenDelayMs),
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
        new WaitCommand(200),
        new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension));
  }

  public Command autoGrabCommand() { // Unused, able to delete
    return new SequentialCommandGroup(
            new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle))
                    .alongWith(new InstantCommand(() -> slideArmServo.setPosition(Goal.GRAB.slideArmPos)))
                    .alongWith(new InstantCommand(() -> wristServo.setPosition(Goal.GRAB.wristPos))),
            new WaitCommand(grabTimeout),
            new InstantCommand(() -> intakeClawServo.setPosition(Goal.GRAB.clawAngle)),
            new WaitCommand(grabTimeout),
            new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos))
                    .alongWith(new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos))),
            new WaitCommand(grabTimeout)
    );
  }

  public Command autoOpenIntakeClaw() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle)),
            new WaitCommand(grabTimeout)
    );
  }


  public void initialize() {
    slideArmServo.setPosition(Goal.AIM.slideArmPos);
    slideRightServo.setPosition(SlideServo.BACK.extensionVal);
    intakeClawServo.setPosition(Goal.HANDOFF.clawAngle);
    wristServo.setPosition(Goal.HANDOFF.wristPos);
    wristTurnServo.setPosition(Goal.HANDOFF.turnAngle);
    handoffWristTurn();
    turnServo = TurnServo.DEG_0;
  }

  public void handoffWristTurn() {
    turnAngleDeg = TurnServo.DEG_0.turnAngleDeg;
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
    slideArmServo.setPosition(Goal.AIM.slideArmPos);
  }

  public void slideArmUp() {
    // This is up for the auto
    slideArmServo.setPosition(0.6);
  }

  public enum Goal {
    STOW(-1, currentRobot==DriveConstants.RobotType.ALPHA?0.4:0.255, currentRobot==DriveConstants.RobotType.ALPHA?0.39:0.47, 0.4, currentRobot==DriveConstants.RobotType.ALPHA?0.5:0.35),
    AIM(slideExtensionVal, currentRobot==DriveConstants.RobotType.ALPHA?0.32:0.63, currentRobot==DriveConstants.RobotType.ALPHA?0.75:0.23, turnAngleDeg, currentRobot==DriveConstants.RobotType.ALPHA?0.5:0.35),
    GRAB(slideExtensionVal, currentRobot==DriveConstants.RobotType.ALPHA?0.47:0.8, currentRobot==DriveConstants.RobotType.ALPHA?0.75:0.23, turnAngleDeg, currentRobot==DriveConstants.RobotType.ALPHA?0.24:0.63),
    HANDOFF(0.22, currentRobot==DriveConstants.RobotType.ALPHA?0.1:0.255, currentRobot==DriveConstants.RobotType.ALPHA?0.39:0.6, 0.4, currentRobot==DriveConstants.RobotType.ALPHA?0.24:0.35);

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
    switch (slideServo) {
      case FRONT:
        slideExtensionVal = SlideServo.FRONT.extensionVal;
        slideServo = SlideServo.FRONT;
        break;
      case MIDDLE:
        slideExtensionVal = SlideServo.FRONT.extensionVal;
        slideServo = SlideServo.FRONT;
        break;
      case BACK:
        slideExtensionVal = SlideServo.MIDDLE.extensionVal;
        slideServo = SlideServo.MIDDLE;
        break;
    }
  }

  public void backwardSlideExtension() {
    switch (slideServo) {
      case FRONT:
        slideExtensionVal = SlideServo.MIDDLE.extensionVal;
        slideServo = SlideServo.MIDDLE;
        break;
      case MIDDLE:
        slideExtensionVal = SlideServo.BACK.extensionVal;
        slideServo = SlideServo.BACK;
        break;
      case BACK:
        slideExtensionVal = SlideServo.BACK.extensionVal;
        slideServo = SlideServo.BACK;
        break;
    }
  }

  public void autoForwardSlideExtension() {
    slideExtensionVal = SlideServo.FRONT.extensionVal;
    slideServo = SlideServo.FRONT;
  }

  public void autoBackSlideExtension() {
    slideExtensionVal = SlideServo.BACK.extensionVal;
    slideServo = SlideServo.BACK;
  }

  private final double preHandoffSlideExtendedVal = 0.25;

  public void preHandoffSlideExtension() {
    slideExtensionVal = preHandoffSlideExtendedVal;
  }

  public boolean isSlideForward() {
    return slideExtensionVal > preHandoffSlideExtendedVal;
  }

  public void leftTurnServo() {
    switch (turnServo) {
      case LEFT_45:
        turnServo = TurnServo.LEFT_45;
        break;
      case DEG_0:
        turnServo = TurnServo.LEFT_45;
        break;
      case RIGHT_45:
        turnServo = TurnServo.DEG_0;
        break;
      case RIGHT_90:
        turnServo = TurnServo.RIGHT_45;
        break;
    }
    setServoPos(turnServo);
  }

  public void rightTurnServo() {
    switch (turnServo) {
      case LEFT_45:
        turnServo = TurnServo.RIGHT_45;
        break;
      case DEG_0:
        turnServo = TurnServo.RIGHT_45;
        break;
      case RIGHT_45:
        turnServo = TurnServo.RIGHT_90;
        break;
      case RIGHT_90:
        turnServo = TurnServo.RIGHT_90;
        break;
    }
    setServoPos(turnServo);
  }

  public void setServoPos(TurnServo pos) {
    turnAngleDeg = pos.turnAngleDeg;
    turnServo = pos;
  }

  private Command setTurnServoPosCommand(TurnServo pos, long delay) {
    return new ConditionalCommand(
        new InstantCommand(
                () -> {
                  setServoPos(pos);
                })
            .andThen(new WaitCommand(delay)),
        new InstantCommand(() -> {}),
        () -> getServoPos() != pos);
  }

  public TurnServo getServoPos() {
    return turnAngleDeg == turnServo.turnAngleDeg ? turnServo : TurnServo.UNKNOWN;
  }

  public enum TurnServo {
    LEFT_45(currentRobot == DriveConstants.RobotType.ALPHA ? 0.25 : 0.715),
    DEG_0(currentRobot == DriveConstants.RobotType.ALPHA ? 0.4 : 0.565),
    RIGHT_45(currentRobot == DriveConstants.RobotType.ALPHA ? 0.55 : 0.45),
    RIGHT_90(currentRobot == DriveConstants.RobotType.ALPHA ? 0.7 : 0.34),
    UNKNOWN(-1);

    private double turnAngleDeg;

    TurnServo(double turnPosition) {
      this.turnAngleDeg = turnPosition;
    }
  }

  enum SlideServo {
    FRONT(currentRobot == DriveConstants.RobotType.ALPHA ? 0.42 : 500),
    MIDDLE(currentRobot == DriveConstants.RobotType.ALPHA ? 0.3 : 350),
    BACK(currentRobot == DriveConstants.RobotType.ALPHA ? 0.21 : 0);

    private double extensionVal;

    SlideServo(double extensionVal) {
      this.extensionVal = extensionVal;
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

    if (currentRobot == DriveConstants.RobotType.DELTA) {
      ((MotorServo)slideRightServo).update();
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
