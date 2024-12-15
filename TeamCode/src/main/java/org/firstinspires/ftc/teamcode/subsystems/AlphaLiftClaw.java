package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AlphaLiftClaw extends SubsystemBase {
  private final Servo liftArmServo;
  private final Servo liftClawServo;
  private final Servo liftWristServo;
  private boolean isClawOpen;

  public AlphaLiftClaw(final HardwareMap hardwareMap) {
    liftArmServo = hardwareMap.get(Servo.class, "liftArmServo"); // 0.3 Up 0.7 Down
    liftClawServo = hardwareMap.get(Servo.class, "liftClawServo"); // 0 Close 0.5 Open
    liftWristServo = hardwareMap.get(Servo.class, "liftWristServo");
  }

  public void initialize() {
    liftWristServo.setPosition(ServoPositions.STOW.liftWristPosition);
    liftArmServo.setPosition(ServoPositions.STOW.liftArmPosition);
    liftClawServo.setPosition(ServoPositions.STOW.liftClawPosition);
  }

  public void grabWrist() {
    liftWristServo.setPosition(0.39);
  }

  public void basketWrist() {
    liftWristServo.setPosition(0.7);
  }

  public void chamberWrist() {
    liftWristServo.setPosition(0.58);
  }

  public void switchLiftClaw() {
    if (isClawOpen) {
      openClaw();
    } else {
      closeClaw();
    }
    isClawOpen = !isClawOpen;
  }

  public void openClaw() {
    liftClawServo.setPosition(0.5);
  }

  public void closeClaw() {
    liftClawServo.setPosition(0.27);
  }

  public void upLiftArm() {
    liftArmServo.setPosition(0.47);
  }

  public void foldLiftArm() {
    liftArmServo.setPosition(0.85);
  }

  public void grabLiftArm() {
    liftArmServo.setPosition(0.19);
  }

  public void chamberLiftArm() {
    liftArmServo.setPosition(0.72);
  }

  public enum ServoPositions {
    STOW(0.85, 0.27, 0.7),
    CHAMBER(0.72, 0.27, 0.58),
    BASKET(0.47, 0.27, 0.7),
    GRAB(0.19, 0.5, 0.39);

    private double liftArmPosition;
    private double liftWristPosition;
    private double liftClawPosition;

    ServoPositions(double liftArmPosition, double liftClawPosition, double liftWristPosition) {
      this.liftArmPosition = liftArmPosition;
      this.liftClawPosition = liftArmPosition;
      this.liftWristPosition = liftWristPosition;
    }
  }
}

/*package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Config
public class AlphaClaw extends SubsystemBase {
    public static double slideServo_ExtendPose = 0.73;
    public static double slideServo_ContractPose = 0.17;
    public static double clawServo_OpenPose = 0.4;
    public static double clawServo_GrabPose = 0.66;
    public static double clawTurnServo_MinPose = 0;
    public static double clawTurnServo_MaxPose = 1;
    private final HardwareMap hardwareMap;
    private final Servo clawServo, clawTurnServo, slideServo;
    private double clawTurnServoSetpoint = clawTurnServo_MinPose;
    public AlphaClaw(HardwareMap hm){
        hardwareMap = hm;
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        clawTurnServo = hardwareMap.get(Servo.class,"clawTurnServo");
        slideServo = hardwareMap.get(Servo.class,"slideServo");
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
}*/
