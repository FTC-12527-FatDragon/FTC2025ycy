package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;
import static org.firstinspires.ftc.teamcode.utils.ServoUtils.setServoPosCommand;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.MotorServo;

@Config
public class AlphaSlide extends MotorPIDSlideSubsystem{

  // aimCommand
  public static long aimCommand_wristTurn2ArmDelayMs = 0;
  public static long aimCommand_Arm2OpenDelayMs = 20;

  private final Servo intakeClawServo, wristServo;
  // <----> 0 DEG_0
  // CCW +
  private final ServoEx wristTurnServo;
  private final Servo slideArmServo, slideRightServo;
  private boolean hasGamepiece = false;

  public static long waitGrabTimeout = 300;
  public static long waitGrabTimeout3 = 400;

  public static long slideRetractFar = currentRobot == DriveConstants.RobotType.ALPHA ? 500 : 400;
  public static long slideRetractNear = 150;
  public static long slideRetractAuto = 500;
  public static long slideExtensionMax = 510;

  private static double turnAngleDeg = 0;
  private TurnServo turnServo = TurnServo.DEG_0;

  public static long grabTimeout = 70;

  public static double resetPower = -0.3;

  @Setter @Getter private Goal goal = Goal.STOW;
//  private boolean isIntakeClawOpen = false;

  private final Telemetry telemetry; // 0 0.5 0.8

  @Getter @Setter private boolean normalHandoff = false;

  @Setter @Getter private SlideServo slideServo = SlideServo.BACK;

  public AlphaSlide(final HardwareMap hardwareMap, final Telemetry telemetry) {
    slideArmServo = hardwareMap.get(Servo.class, "slideArmServo"); // 0.5 up 0.9 half 1 down


    if(currentRobot== DriveConstants.RobotType.ALPHA){
      slideRightServo = hardwareMap.get(Servo.class, "slideLeftServo");
    }else{
      slideRightServo = new MotorServo(hardwareMap, "slideMotor");//
      if(currentRobot == DriveConstants.RobotType.EPSILON){
        slideRightServo.setDirection(Servo.Direction.REVERSE);
      }
    }

    intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo"); // 0.3 close 0.7 open
    wristServo = hardwareMap.get(Servo.class, "wristServo"); // 0.05 up 0.75 down

    wristTurnServo = new SimpleServo(hardwareMap, "wristTurnServo", 0, 180);
    if(currentRobot == DriveConstants.RobotType.ALPHA){
      wristTurnServo.setInverted(true);
      wristTurnServo.setRange(-176.3408304498271, 119.506920415225);
    }else{
      wristTurnServo.setRange(0, 0); // TODO: finish this
    }

    this.telemetry = telemetry;
    goal = Goal.STOW;
    telemetry.addData("Current State", goal);
    telemetry.addData("slide pos", slideRightServo.getPosition());
    telemetry.update();
  }

  public Command setGoalCommand(Goal newGoal) {
    return new InstantCommand(() -> goal = newGoal);
  }

  public Command aimCommand(TurnServo turnServoPos) {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.AIM),
        new InstantCommand(() -> wristServo.setPosition(Goal.AIM.wristPos)),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle)),
        setTurnServoPosCommand(turnServoPos, aimCommand_wristTurn2ArmDelayMs),
        setServoPosCommand(slideArmServo, Goal.AIM.slideArmPos, aimCommand_Arm2OpenDelayMs)
    );
  }

  public Command openClawCommand() {
    return setServoPosCommand(intakeClawServo, intakeClawServo_Open, grabTimeout);
  }

  public Command aimCommand() {
    return aimCommand(TurnServo.DEFAULT);
  }

  public Command grabCommand(TurnServo turnServoPos) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> goal = Goal.GRAB),
            setTurnServoPosCommand(turnServoPos, aimCommand_wristTurn2ArmDelayMs) // Won't do anything if turnServoPos is invalid
                    .alongWith(
                            setServoPosCommand(slideArmServo, Goal.GRAB.slideArmPos, grabTimeout)
                    ),
            setServoPosCommand(intakeClawServo, Goal.GRAB.clawAngle, grabTimeout),
            new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
            new InstantCommand(() -> goal = Goal.STOW));
  }

  public Command grabCommand() {
    return grabCommand(TurnServo.DEFAULT);
  }

  public Command handoffCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal = Goal.HANDOFF),
        setTurnServoPosCommand(TurnServo.DEG_0, aimCommand_wristTurn2ArmDelayMs).alongWith(
                  setServoPosCommand(wristServo, Goal.HANDOFF.wristPos, 100),
                  setServoPosCommand(slideArmServo, Goal.HANDOFF.slideArmPos, 150)
        ),
        new InstantCommand(() -> slideServo = SlideServo.HANDOFF).andThen(
                  new ConditionalCommand(
                          new WaitCommand(slideRetractFar),
                          new WaitCommand(slideRetractNear),
                          () -> slideServo.extensionVal >= SlideServo.MIDDLE.extensionVal
                  )
        )
    );
  }

  public Command autoGrabCommand() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle))
                    .alongWith(new InstantCommand(() -> slideArmServo.setPosition(Goal.GRAB.slideArmPos)))
                    .alongWith(new InstantCommand(() -> wristServo.setPosition(Goal.GRAB.wristPos))),
            new WaitCommand(waitGrabTimeout3),
            new InstantCommand(() -> intakeClawServo.setPosition(Goal.GRAB.clawAngle)),
            new WaitCommand(grabTimeout),
            new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos))
                    .alongWith(new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos))),
            new WaitCommand(grabTimeout)
    );
  }

  public Command autoGrabCommand3A() {
    return new SequentialCommandGroup(
            setTurnServoPosCommand(TurnServo.RIGHT_90, 200),
            new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle))
                    .alongWith(new InstantCommand(() -> slideArmServo.setPosition(Goal.GRAB.slideArmPos)))
                    .alongWith(new InstantCommand(() -> wristServo.setPosition(Goal.GRAB.wristPos)))
//            new InstantCommand(() -> wristTurnServo.setPosition(TurnServo.RIGHT_90.turnAngleDeg)),
    );
  }

  public Command autoGrabCommand3B() {
    return new SequentialCommandGroup(
            new WaitCommand(waitGrabTimeout),
            new InstantCommand(() -> intakeClawServo.setPosition(Goal.GRAB.clawAngle)),
            new WaitCommand(grabTimeout),
            setTurnServoPosCommand(TurnServo.DEG_0, 200),
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
    slideArmServo.setPosition(Goal.STOW.slideArmPos);
    if(currentRobot== DriveConstants.RobotType.EPSILON){
      ((MotorServo)slideRightServo).resetEncoder();
    }
    slideRightServo.setPosition(SlideServo.BACK.extensionVal);
    intakeClawServo.setPosition(Goal.HANDOFF.clawAngle);
    wristServo.setPosition(Goal.HANDOFF.wristPos);
    wristTurnServo.setPosition(Goal.HANDOFF.turnAngle);
    handoffWristTurn();
    setTurnServoPos(TurnServo.DEG_0);
  }

  public void handoffWristTurn() {
    turnAngleDeg = TurnServo.DEG_0.turnAngleDeg;
  }

  public void openIntakeClaw() {
    intakeClawServo.setPosition(Goal.STOW.clawAngle);
//    isIntakeClawOpen = true;
  }

  public void closeIntakeClaw() {
    intakeClawServo.setPosition(Goal.HANDOFF.clawAngle);
//    isIntakeClawOpen = false;
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

  public static double slideArmServo_Down = currentRobot==DriveConstants.RobotType.ALPHA?0.5:0.76;
  public static double intakeClawServo_Open = currentRobot==DriveConstants.RobotType.ALPHA?0.2:0.5;
  public static double intakeClawServo_Close = currentRobot==DriveConstants.RobotType.ALPHA?0.635:0.76;

  @Override
  void runOpenLoop(double percent) {
    double output = Range.clip(percent, -1, 1);
    ((MotorServo)slideRightServo).setPower(output);
  }

  @Override
  long getCurrentPosition() {
    return (long)((MotorServo)slideRightServo).getPosition();
  }

  public boolean isSlideMotorZeroed() {
    return MathUtils.isNear(0, getCurrentPosition(), 10);
  }

  @Override
  double getResetPower() {
    return resetPower;
  }

  @Override
  void resetEncoder() {
    ((MotorServo)slideRightServo).resetEncoder();
  }
//  public static double slideArmServo_PreGrab = 0.45;

  public enum Goal {
    STOW(-1,                currentRobot==DriveConstants.RobotType.ALPHA?0.4:0.3,  currentRobot==DriveConstants.RobotType.ALPHA?0.39:0.5, 0.4,          intakeClawServo_Open),
    AIM(-1,  currentRobot==DriveConstants.RobotType.ALPHA?0.35:0.6,  currentRobot==DriveConstants.RobotType.ALPHA?0.75:0.04, turnAngleDeg, intakeClawServo_Open),
    GRAB(-1, slideArmServo_Down                                    ,  currentRobot==DriveConstants.RobotType.ALPHA?0.75:0.04, turnAngleDeg, intakeClawServo_Close),
    HANDOFF(-1,           currentRobot==DriveConstants.RobotType.ALPHA?0.11:0.14, currentRobot==DriveConstants.RobotType.ALPHA?0.45:0.47,  0.4,          intakeClawServo_Close);
    //Arm, Wrist, Wrist Turn
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
        slideServo = SlideServo.FRONT;
        break;
      case MIDDLE:
        slideServo = SlideServo.FRONT;
        break;
      case HANDOFF:
      case BACK:
        slideServo = SlideServo.MIDDLE;
        break;
      default:
        telemetry.addLine("Warning: Unprocessed state: " + slideServo);
    }
    telemetry.addLine("SLIDESERVO FORWARD");
  }

  public void backwardSlideExtension() {
    switch (slideServo) {
      case FRONT:
        slideServo = SlideServo.MIDDLE;
        break;
      case MIDDLE:
        slideServo = SlideServo.BACK;
        break;
      case HANDOFF:
      case BACK:
        slideServo = SlideServo.BACK;
        break;
      default:
        telemetry.addLine("Warning: Unprocessed state: " + slideServo);
    }
    telemetry.addLine("SLIDESERVO BACKWARD");
  }

  public void autoForwardSlideExtension() {
    slideServo = SlideServo.FRONT;
  }

  public void autoBackSlideExtension() {
    slideServo = SlideServo.BACK;
  }

  public void preHandoffSlideExtension() {
    slideServo = SlideServo.PRE_HANDOFF;
  }

  public boolean isSlideForward() {
    return slideServo.extensionVal > SlideServo.PRE_HANDOFF.extensionVal;
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
    setTurnServoPos(turnServo);
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
    setTurnServoPos(turnServo);
  }

  public void setTurnServoPos(TurnServo pos) {
    turnAngleDeg = pos.turnAngleDeg;
    turnServo = pos;
  }

  public void setTurnServoPos(double desiredRotDeg) {
    wristTurnServo.turnToAngle(desiredRotDeg);
    turnAngleDeg = wristTurnServo.getPosition();
  }

  public Command setTurnServoPosCommand(TurnServo pos, long delay) {
    return new ConditionalCommand(
        new InstantCommand(
                () -> {
                  setTurnServoPos(pos);
                })
            .andThen(new WaitCommand(delay)),
        new InstantCommand(() -> {}),
        () -> getServoPos() != pos && pos.turnAngleDeg>=0);
  }

  public TurnServo getServoPos() {
    return turnAngleDeg == turnServo.turnAngleDeg ? turnServo : TurnServo.UNKNOWN;
  }

  public enum TurnServo {
    LEFT_55(currentRobot == DriveConstants.RobotType.ALPHA ? 0.8 : 0.7),
    LEFT_45(currentRobot == DriveConstants.RobotType.ALPHA ? 0.75 : 0.65),
    DEG_0(currentRobot == DriveConstants.RobotType.ALPHA ? 0.59 : 0.55),
    RIGHT_45(currentRobot == DriveConstants.RobotType.ALPHA ? 0.45 : 0.45),
    RIGHT_90(currentRobot == DriveConstants.RobotType.ALPHA ? 0.29 : 0.315),
    UNKNOWN(-1),
    DEFAULT(-1);

    private double turnAngleDeg;

    TurnServo(double turnPosition) {
      this.turnAngleDeg = turnPosition;
    }
  }

  public enum SlideServo {
    FRONT(currentRobot == DriveConstants.RobotType.ALPHA ? 0.395 : slideExtensionMax),
    MIDDLE(currentRobot == DriveConstants.RobotType.ALPHA ? 0.275 : slideExtensionMax*0.5),
    PRE_HANDOFF(currentRobot == DriveConstants.RobotType.ALPHA ? 0.225: 25),//slideExtensionMax*0.1),
    HANDOFF(currentRobot == DriveConstants.RobotType.ALPHA ? 0.19 : 20),
    BACK(currentRobot == DriveConstants.RobotType.ALPHA ? 0.19 : 0);

    private double extensionVal;

    SlideServo(double extensionVal) {
      this.extensionVal = extensionVal;
    }
  }

  @Override
  public void periodic() {

    if (wristTurnServo != null) {
      wristTurnServo.setPosition(Range.clip(turnAngleDeg, 0, 1));

      if (currentRobot == DriveConstants.RobotType.ALPHA)
        slideRightServo.setPosition(Range.clip(slideServo.extensionVal, 0, 1));
      else {
        slideRightServo.setPosition(slideServo.extensionVal);
        if(!isResetting){
          ((MotorServo) slideRightServo).update();
        }
      }

      telemetry.addData("Slide.Current State", goal);
      telemetry.addData("Slide.Current State.Is at handoff", goal == Goal.HANDOFF);
      telemetry.addData("Slide.Claw Position", intakeClawServo.getPosition());
      telemetry.addData("Slide.Extension", slideServo.extensionVal);
      telemetry.addData("Slide.Turn Angle", turnAngleDeg);
      telemetry.addData("Slide.Servo Position", slideRightServo.getPosition());
    }
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
