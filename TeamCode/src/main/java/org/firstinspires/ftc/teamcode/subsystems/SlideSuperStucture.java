package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.WaitLambdaCommand;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

import lombok.Getter;

public class SlideSuperStucture extends SubsystemBase {
    private final Servo intakeClawServo, wristServo, wristTurnServo;
    private final Servo slideArmServo, slideLeftServo, slideRightServo;
    AnalogInput /*slideLeftEncoder,*/ slideRightEncoder;
    private boolean hasGamepiece = false;
    private static double slideExtensionVal = 0.675;

    private static double turnAngleDeg = 0;
    private TurnServo turnServo = TurnServo.DEG_0;

    @Getter private Goal goal = Goal.STOW;
    private boolean isIntakeClawOpen = false;

    private final Telemetry telemetry; //0 0.5 0.8

    public SlideSuperStucture(final HardwareMap hardwareMap, final Telemetry telemetry) {
        slideArmServo = hardwareMap.get(Servo.class, "slideArmServo"); //0.5 up 0.9 half 1 down

        slideLeftServo = hardwareMap.get(Servo.class, "slideLeftServo");
        slideLeftServo.setDirection(Servo.Direction.REVERSE);
//        slideLeftEncoder = hardwareMap.get(AnalogInput.class, "slideLeftEncoder");

        slideRightServo = hardwareMap.get(Servo.class, "slideRightServo"); //1 stow
        slideRightEncoder = hardwareMap.get(AnalogInput.class, "slideRightEncoder");

        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");// 0.3 close 0.7 open
        wristServo = hardwareMap.get(Servo.class, "wristServo"); //0.05 up 0.75 down

        wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");
        this.telemetry = telemetry;
        goal = Goal.STOW;
        telemetry.addData("Current State", goal);
        telemetry.update();

    }

    public Command aimCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> goal = Goal.AIM),
                new InstantCommand(() ->
                {
                    turnAngleDeg = 0;
                    turnServo = TurnServo.DEG_0;
                }),
                new WaitCommand(100),
                new InstantCommand(() -> slideArmServo.setPosition(Goal.AIM.slideArmPos)),
                new WaitCommand(100),
                new InstantCommand(() -> wristServo.setPosition(Goal.AIM.wristPos)),
                new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle))
        );
    }

    public Command grabCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> goal = Goal.GRAB),
                new InstantCommand(() -> slideArmServo.setPosition(Goal.GRAB.slideArmPos)),
                new WaitCommand(100),
                new InstantCommand(() -> intakeClawServo.setPosition(Goal.GRAB.clawAngle)),
                new WaitCommand(100),
                new InstantCommand(() -> slideArmServo.setPosition(Goal.AIM.slideArmPos)),
                new InstantCommand(() -> goal = Goal.AIM)
        );
    }

    public Command handoffCommand(boolean isOpen) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> goal = Goal.HANDOFF),
                new InstantCommand(() ->
                {
                    turnAngleDeg = 0;
                    turnServo = TurnServo.DEG_0;
                }),
                new WaitCommand(100),
                new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos)),
                new WaitCommand(200),
                new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
                isOpen?new WaitCommand(300):new WaitUntilCommand(this::slideAtSetpoint),
                new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension)
        );
    }

    public Command handoffCommand() {
        return handoffCommand(false);
    }

    public void openIntakeClaw() {
        intakeClawServo.setPosition(0.7);
        isIntakeClawOpen = true;
    }

    public void closeIntakeClaw() {
        intakeClawServo.setPosition(0);
        isIntakeClawOpen = false;
    }

    public enum Goal {
        STOW(0.95, 0, 0, 0, 0.6),
        AIM(slideExtensionVal, 0.85 , 0.75, turnAngleDeg, 0.6),
        GRAB(slideExtensionVal, 1, 0.75, turnAngleDeg, 0.3),
        HANDOFF(0.925, 0.5, 0.05, 0, 0.3);

        private final double slideExtension;
        private final double slideArmPos;
        private final double wristPos;
        private final double turnAngle;
        private final double clawAngle;
        Goal(double slideExtension, double slideArmPos, double wristPos, double turnAngle, double clawAngle) {
            this.slideExtension = slideExtension;
            this.slideArmPos = slideArmPos;
            this.wristPos = wristPos;
            this.turnAngle = turnAngle;
            this.clawAngle = clawAngle;
        }
    }

    public void forwardSlideExtension() {
        slideExtensionVal += 0.05;
    }

    public void backwardSlideExtension() {
        slideExtensionVal -= 0.05;
    }

    public void leftTurnServo() {
        switch (turnServo) {
            case DEG_0:
                turnAngleDeg = 0.5;
                turnServo = TurnServo.DEG_05;
                break;
            case DEG_05:
                turnAngleDeg = 0.8;
                turnServo = TurnServo.DEG_08;
                break;
            case DEG_08:
                break;
        }
    }

    public void rightTurnServo() {
        switch (turnServo) {
            case DEG_0:
                break;
            case DEG_05:
                turnAngleDeg = 0;
                turnServo = TurnServo.DEG_0;
                break;
            case DEG_08:
                turnAngleDeg = 0.5;
                turnServo = TurnServo.DEG_05;
                break;
        }
    }

    public boolean slideAtSetpoint(){
        // Prob. rot [0-1] -> [0-360]?
        return MathUtils.isNear(getSlidePosition(), slideExtensionVal * -0.58095238095239393939393939393939 + 0.70137085137084848484848484848485, 0.01);
    }

    public double getSlidePosition(){
        return slideRightEncoder.getVoltage() / slideRightEncoder.getMaxVoltage();
    }


    enum TurnServo {
        DEG_0,
        DEG_05,
        DEG_08
    }

    @Override
    public void periodic() {

        wristTurnServo.setPosition(Range.clip(turnAngleDeg, 0, 1));
        slideRightServo.setPosition(Range.clip(slideExtensionVal, 0, 1));


        telemetry.addData("Current State", goal);
        telemetry.addData("Bur Gemen", goal == Goal.HANDOFF);
        telemetry.addData("Claw Position", intakeClawServo.getPosition());
        telemetry.addData("Slide Extension", slideExtensionVal);
        telemetry.addData("Turn Angle", turnAngleDeg);
        telemetry.addData("Slide Position", getSlidePosition());
        telemetry.addData("Slide Setpoint", slideExtensionVal);
        telemetry.update();
    }

}
