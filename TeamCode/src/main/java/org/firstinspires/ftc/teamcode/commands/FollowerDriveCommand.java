package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.drive.FollowerDrive;

public class FollowerDriveCommand extends CommandBase {
    FollowerDrive followerDrive;
    GamepadEx gamepad;


    public FollowerDriveCommand(FollowerDrive followerDrive, GamepadEx gamepad) {
        this.followerDrive = followerDrive;
        this.gamepad = gamepad;

        addRequirements(followerDrive);
    }

    @Override
    public void execute() {
        followerDrive.setTeleOpMovementVectors(gamepad);
    }
}
