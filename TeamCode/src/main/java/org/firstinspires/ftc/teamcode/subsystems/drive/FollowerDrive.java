package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drive.constants.FConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.constants.LConstants;

public class FollowerDrive implements Subsystem {
    private double speed = 1;
    private int leftYDirection = 1, leftXDirection = 1, rightXDirection = 1;
    private boolean robotCentric = true;
    private final Follower follower;

    public FollowerDrive(HardwareMap hardwareMap) {
        this.follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
    }

    public void setTeleOpMovementVectors(GamepadEx gamepad) {
        follower.setTeleOpMovementVectors(leftYDirection * speed * gamepad.getLeftY(), leftXDirection * speed * gamepad.getLeftX(), rightXDirection * speed * gamepad.getRightX(), robotCentric);
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }
}