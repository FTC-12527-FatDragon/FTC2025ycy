package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.isReflected;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.kV;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import lombok.Getter;
import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.GoBildaLocalizer;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.lib.roadrunner.util.LynxModuleUtil;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive implements Subsystem {
  public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0.003);
  public static PIDCoefficients HEADING_PID = new PIDCoefficients(13, 0, 0.07);

  public static double LATERAL_MULTIPLIER = 1.4514;

  public static double VX_WEIGHT = 1;
  public static double VY_WEIGHT = 1;
  public static double OMEGA_WEIGHT = 1;

  public static double ADMISSIBLE_TIMEOUT = 0.5;

  private TrajectorySequenceRunner trajectorySequenceRunner;

  @Getter
  private static final TrajectoryVelocityConstraint VEL_CONSTRAINT =
      getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);

  @Getter
  private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT =
      getAccelerationConstraint(MAX_ACCEL);

  private TrajectoryFollower follower;

  private DcMotorEx leftFront, leftRear, rightRear, rightFront;
  private List<DcMotorEx> motors;

  private GoBildaLocalizer od;
  private VoltageSensor batteryVoltageSensor;

  private List<Integer> lastEncPositions = new ArrayList<>();
  private List<Integer> lastEncVels = new ArrayList<>();

  private double yawHeading = 0;

  public SampleMecanumDrive(HardwareMap hardwareMap) {
    super(
        kV,
        kA,
        kStatic,
        TRACK_WIDTH,
        TRACK_WIDTH,
        LATERAL_MULTIPLIER); // Drive Constants are passed to here

    follower =
        new HolonomicPIDVAFollower(
            TRANSLATIONAL_PID,
            TRANSLATIONAL_PID,
            HEADING_PID,
            new Pose2d(1.5, 1.5, Math.toRadians(2)), // Pose Error
            ADMISSIBLE_TIMEOUT);

    LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    // TODO: adjust the names of the following hardware devices to match your configuration
    od = new GoBildaLocalizer(hardwareMap, DriveConstants.GoBildaLocalizerPerpendicularOffset);

    leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
    leftRear = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
    rightRear = hardwareMap.get(DcMotorEx.class, "rightBackMotor");
    rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
    if (isReflected) {
      leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
      leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
      rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
      rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    } else {
      leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
      leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
      rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
      rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

    for (DcMotorEx motor : motors) {
      MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
      motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
      motor.setMotorType(motorConfigurationType);
    }

    if (RUN_USING_ENCODER) {
      setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
      setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
    }

    // TODO: reverse any motors using DcMotor.setDirection()

    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();

    // TODO: if desired, use setLocalizer() to change the localization method
    setLocalizer(od);

    trajectorySequenceRunner =
        new TrajectorySequenceRunner(
            follower,
            HEADING_PID,
            batteryVoltageSensor,
            lastEncPositions,
            lastEncVels,
            lastTrackingEncPositions,
            lastTrackingEncVels);

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
    return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
    return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
    return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
    return new TrajectorySequenceBuilder(
        startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT, MAX_ANG_VEL, MAX_ANG_ACCEL);
  }

  public void turnAsync(double angle) {
    trajectorySequenceRunner.followTrajectorySequenceAsync(
        trajectorySequenceBuilder(getPoseEstimate()).turn(angle).build());
  }

  public void turn(double angle) {
    turnAsync(angle);
    waitForIdle();
  }

  public void followTrajectoryAsync(Trajectory trajectory) {
    trajectorySequenceRunner.followTrajectorySequenceAsync(
        trajectorySequenceBuilder(trajectory.start()).addTrajectory(trajectory).build());
  }

  public void followTrajectory(Trajectory trajectory) {
    followTrajectoryAsync(trajectory);
    waitForIdle();
  }

  public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
    trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
  }

  public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
    followTrajectorySequenceAsync(trajectorySequence);
    waitForIdle();
  }

  public Pose2d getLastError() {
    return trajectorySequenceRunner.getLastPoseError();
  }

  public void update() {
    updatePoseEstimate();
    DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
    if (signal != null) setDriveSignal(signal);
  }

  public void waitForIdle() {
    while (!Thread.currentThread().isInterrupted() && isBusy()) update();
  }

  public boolean isBusy() {
    return trajectorySequenceRunner.isBusy();
  }

  public void setMode(DcMotor.RunMode runMode) {
    for (DcMotorEx motor : motors) {
      motor.setMode(runMode);
    }
  }

  public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
    for (DcMotorEx motor : motors) {
      motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
  }

  public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
    PIDFCoefficients compensatedCoefficients =
        new PIDFCoefficients(
            coefficients.p,
            coefficients.i,
            coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.getVoltage());

    for (DcMotorEx motor : motors) {
      motor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
  }

  public void setFieldRelativeDrivePower(Pose2d drivePower) {
    Pose2d vel = drivePower;
    double botHeading = od.getHeading() - yawHeading;

    // Rotate the movement direction counter to the bot's rotation
    double rotX = vel.getX() * Math.cos(-botHeading) - vel.getY() * Math.sin(-botHeading);
    double rotY = vel.getX() * Math.sin(-botHeading) + vel.getY() * Math.cos(-botHeading);

    double denom = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(drivePower.getHeading()), 1);
    vel = new Pose2d(rotX, rotY, drivePower.getHeading()).div(denom);

    setDrivePower(vel);
  }

  public double getHeading() {
    return od.getHeading() - yawHeading;
  }

  public void resetHeading() {
    yawHeading = od.getHeading();
  }

  public void setWeightedDrivePower(Pose2d drivePower) {
    Pose2d vel = drivePower;

    if (Math.abs(drivePower.getX())
            + Math.abs(drivePower.getY())
            + Math.abs(drivePower.getHeading())
        > 1) {
      // re-normalize the powers according to the weights
      double denom =
          VX_WEIGHT * Math.abs(drivePower.getX())
              + VY_WEIGHT * Math.abs(drivePower.getY())
              + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

      vel =
          new Pose2d(
                  VX_WEIGHT * drivePower.getX(),
                  VY_WEIGHT * drivePower.getY(),
                  OMEGA_WEIGHT * drivePower.getHeading())
              .div(denom);
    }

    setDrivePower(vel);
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() { // Is it correct?
    lastEncPositions.clear();

    List<Double> wheelPositions = new ArrayList<>();
    for (DcMotorEx motor : motors) {
      int position = motor.getCurrentPosition();
      lastEncPositions.add(position);
      wheelPositions.add(encoderTicksToInches(position));
    }
    return wheelPositions;
  }

  @Override
  public List<Double> getWheelVelocities() {
    lastEncVels.clear();

    List<Double> wheelVelocities = new ArrayList<>();
    for (DcMotorEx motor : motors) {
      int vel = (int) motor.getVelocity();
      lastEncVels.add(vel);
      wheelVelocities.add(encoderTicksToInches(vel));
    }
    return wheelVelocities;
  }

  @Override
  public void setMotorPowers(double v, double v1, double v2, double v3) {
    leftFront.setPower(v);
    leftRear.setPower(v1);
    rightRear.setPower(v2);
    rightFront.setPower(v3);
  }

  @Override
  public double getRawExternalHeading() {
    return od.getHeading();
  }

  @Override
  public Double getExternalHeadingVelocity() {
    return od.getHeadingVelocity();
  }

  public static TrajectoryVelocityConstraint getVelocityConstraint(
      double maxVel, double maxAngularVel, double trackWidth) {
    return new MinVelocityConstraint(
        Arrays.asList(
            new AngularVelocityConstraint(maxAngularVel),
            new MecanumVelocityConstraint(maxVel, trackWidth)));
  }

  public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
    return new ProfileAccelerationConstraint(maxAccel);
  }
}
