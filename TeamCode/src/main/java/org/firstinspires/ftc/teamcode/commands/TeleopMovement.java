package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;


public class TeleopMovement extends CommandBase {
    private final Follower follower;
    private GamepadEx gamepad;
    private double speed = 1;
    private int leftYDirection = 1, leftXDirection = 1, rightXDirection = 1;
    private boolean robotCentric = true;

    public TeleopMovement(Follower follower, boolean robotCentric, GamepadEx gamepad, double speed, int forwardMovementDirection, int lateralMovementDirection, int angularMovementDirection) {
        this.follower = follower;
        this.robotCentric = robotCentric;
        this.gamepad = gamepad;
        this.speed = speed;
        this.leftYDirection = forwardMovementDirection;
        this.leftXDirection = lateralMovementDirection;
        this.rightXDirection = angularMovementDirection;
    }

    public TeleopMovement(Follower follower, boolean robotCentric, GamepadEx gamepad, double speed) {
        this(follower, robotCentric, gamepad, speed, 1, 1, 1);
    }

    public TeleopMovement(Follower follower, boolean robotCentric, GamepadEx gamepad) {
        this(follower, robotCentric, gamepad, 1, 1,1,1);
    }

    public TeleopMovement(Follower follower, GamepadEx gamepad, double speed) {
        this(follower,true, gamepad, speed, 1, 1, 1);
    }

    public TeleopMovement(Follower follower, boolean robotCentric, GamepadEx gamepad, int forwardMovementDirection, int lateralMovementDirection, int angularMovementDirection) {
        this(follower, robotCentric, gamepad, 1, forwardMovementDirection, lateralMovementDirection, angularMovementDirection);
    }

    public TeleopMovement(Follower follower, GamepadEx gamepad, double speed, int forwardMovementDirection, int lateralMovementDirection, int angularMovementDirection) {
        this(follower,true, gamepad, speed, forwardMovementDirection, lateralMovementDirection, angularMovementDirection);
    }

    public TeleopMovement setJoystickDirections(int forwardMovementDirection, int lateralMovementDirection, int angularMovementDirection) {
        this.leftYDirection = forwardMovementDirection;
        this.leftXDirection = lateralMovementDirection;
        this.rightXDirection = angularMovementDirection;
        return this;
    }

    public TeleopMovement setRobotCentric() {
        this.robotCentric = true;
        return this;
    }

    public TeleopMovement setFieldCentric() {
        this.robotCentric = false;
        return this;
    }

    public TeleopMovement setSpeed(double speed) {
        this.speed = speed;
        return this;
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
    }

    @Override
    public void execute() {
        follower.setTeleOpMovementVectors(leftYDirection * speed * gamepad.getLeftY(), leftXDirection * speed * gamepad.getLeftX(), rightXDirection * speed * gamepad.getRightX(), robotCentric);
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

