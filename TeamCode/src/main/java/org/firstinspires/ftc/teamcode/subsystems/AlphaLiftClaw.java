package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.currentRobot;
import static org.firstinspires.ftc.teamcode.utils.ServoUtils.setServoPosCommand;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;

@Config
public class AlphaLiftClaw extends SubsystemBase {
  private final Servo liftArmServo;
  private final Servo liftClawServo;
  private final Servo liftWristServo;
  private final Telemetry telemetry;

  public static double LiftClaw_Open = currentRobot==DriveConstants.RobotType.ALPHA?0.8:0.35;
  public static double LiftClaw_Close = currentRobot==DriveConstants.RobotType.ALPHA?0.341:0.575;
  public static double liftArmClimbPos = currentRobot==DriveConstants.RobotType.ALPHA ? 0.38 : 0.43;


  public static long LiftClaw_SwitchDelay = 50;
  public static long LiftArm_Handoff2BackwardGrabDelay = currentRobot==DriveConstants.RobotType.ALPHA?500:1000;

  public AlphaLiftClaw(final HardwareMap hardwareMap, Telemetry telemetry) {
    liftArmServo = hardwareMap.get(Servo.class, "liftArmServo"); // 0.3 Up 0.7 Down
    liftClawServo = hardwareMap.get(Servo.class, "liftClawServo"); // 0 Close 0.5 Open
    liftWristServo = hardwareMap.get(Servo.class, "liftWristServo");
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  }

  public void initialize() {
    liftWristServo.setPosition(ServoPositions.STOW.liftWristPosition);
    liftArmServo.setPosition(ServoPositions.STOW.liftArmPosition);
    liftClawServo.setPosition(ServoPositions.STOW.liftClawPosition);
  }

  public void autoInitialize() {
    liftWristServo.setPosition(ServoPositions.STOW.liftWristPosition);
    liftArmServo.setPosition(ServoPositions.STOW.liftArmPosition);
    liftClawServo.setPosition(ServoPositions.STOW.liftClawPosition);
  }

  public void grabWrist() {
    liftWristServo.setPosition(ServoPositions.GRAB.liftWristPosition);
  }

  public void basketWrist() {
    liftWristServo.setPosition(ServoPositions.BASKET.liftWristPosition);
  }

  public void stowWrist() {
    liftWristServo.setPosition(ServoPositions.STOW.liftWristPosition);
  }

  public void chamberWrist() {
    liftWristServo.setPosition(ServoPositions.CHAMBER.liftWristPosition);
  }

  public Command switchLiftClawCommand() {
    return new ConditionalCommand(
            openClawCommand(),
            closeClawCommand(),
            () -> Math.abs(liftClawServo.getPosition() - LiftClaw_Close) < Math.abs(liftClawServo.getPosition() - LiftClaw_Open)//liftClawServo.getPosition() != ServoPositions.GRAB.liftClawPosition
    );
  }

  public void debugClaw() {
    telemetry.addData("liftClawPos", liftClawServo.getPosition());
  }

  public void openClaw() {
    liftClawServo.setPosition(ServoPositions.GRAB.liftClawPosition);
  }

  public Command openClawCommand(long delay) {
    return setServoPosCommand(liftClawServo, LiftClaw_Open, delay);
  }

  public void climbArm() {
    liftArmServo.setPosition(liftArmClimbPos);
  }

  public Command openClawCommand() {
    return openClawCommand(LiftClaw_SwitchDelay);
  }

  public void closeClaw() {
    liftClawServo.setPosition(ServoPositions.STOW.liftClawPosition);
  }

  public Command closeClawCommand(long delay) {
    return setServoPosCommand(liftClawServo, ServoPositions.STOW.liftClawPosition, delay);
  }

  public Command closeClawCommand() {
    return closeClawCommand(LiftClaw_SwitchDelay);
  }

  public void upLiftArm() {
    liftArmServo.setPosition(ServoPositions.BASKET.liftArmPosition);
  }

  public void foldLiftArm() {
    liftArmServo.setPosition(ServoPositions.STOW.liftArmPosition);
  }

  public Command foldLiftArmCommand(long delay) {
    return setServoPosCommand(liftArmServo, ServoPositions.STOW.liftArmPosition, delay);
  }

  public Command foldLiftArmCommand() {
    return foldLiftArmCommand(200);
  }

  public void grabLiftArm() {
    liftArmServo.setPosition(ServoPositions.GRAB.liftArmPosition);
  }

  public void chamberLiftArm() {
    liftArmServo.setPosition(ServoPositions.CHAMBER.liftArmPosition);
  }

  public enum ServoPositions {
    STOW(currentRobot==DriveConstants.RobotType.ALPHA?0.76:0.84, LiftClaw_Close, currentRobot==DriveConstants.RobotType.ALPHA?0.64:0.64),
    CHAMBER(currentRobot==DriveConstants.RobotType.ALPHA?0.58:0.65, LiftClaw_Close, currentRobot==DriveConstants.RobotType.ALPHA?0.27:0.47),
    BASKET(currentRobot==DriveConstants.RobotType.ALPHA?0.37:0.43, LiftClaw_Close, currentRobot==DriveConstants.RobotType.ALPHA?0.35:0.57),
    GRAB(currentRobot==DriveConstants.RobotType.ALPHA?0.15:0.198, LiftClaw_Open, currentRobot==DriveConstants.RobotType.ALPHA?0.08:0.35);
    //liftarm,liftclaw,liftwrist

    private final double liftArmPosition;
    private final double liftWristPosition;
    private final double liftClawPosition;

    ServoPositions(double liftArmPosition, double liftClawPosition, double liftWristPosition) {
      this.liftArmPosition = liftArmPosition;
      this.liftClawPosition = liftClawPosition;
      this.liftWristPosition = liftWristPosition;
    }
  }

  public boolean getLiftClawPos() {
    return Math.abs(liftClawServo.getPosition() - LiftClaw_Close) < Math.abs(liftClawServo.getPosition() - LiftClaw_Open);
  }

  @Override
  public void periodic() {
    telemetry.addData("Lift Arm Position", liftArmServo.getPosition());
    telemetry.addData("lift claw pos", getLiftClawPos());
    telemetry.update();
  }
}
