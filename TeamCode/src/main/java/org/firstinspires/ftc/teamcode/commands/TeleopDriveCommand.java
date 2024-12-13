package org.firstinspires.ftc.teamcode.commands;

import android.service.controls.actions.BooleanAction;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

public class TeleopDriveCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotate;
    private final DoubleSupplier strafe;
    private final BooleanSupplier shouldReset;
    private final BooleanSupplier shouldSlow;
    private final BooleanSupplier isHeadless;

    public TeleopDriveCommand(
            SampleMecanumDrive drive,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate,
            BooleanSupplier shouldReset,
            BooleanSupplier shouldSlow,
            BooleanSupplier isHeadless) {
        this.drive = drive;
        this.forward = forward;
        this.rotate = rotate;
        this.strafe = strafe;
        this.shouldReset = shouldReset;
        this.shouldSlow = shouldSlow;
        this.isHeadless = isHeadless;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (shouldReset.getAsBoolean()) {
            drive.resetHeading();
        }

        double forwardValue = forward.getAsDouble();
        double strafeValue = strafe.getAsDouble();
        double rotateValue = rotate.getAsDouble();

        if (shouldSlow.getAsBoolean()) {
            forwardValue *= 0.3;
            strafeValue *= 0.3;
            rotateValue *= 0.3;
        }

        Pose2d drivePower = new Pose2d(forwardValue, strafeValue, rotateValue);
        if (isHeadless.getAsBoolean()){
            drive.setFieldRelativeDrivePower(drivePower);
        }else{
            drive.setWeightedDrivePower(drivePower);
        }

    }
}
