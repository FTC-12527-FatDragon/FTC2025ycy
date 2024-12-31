package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class EmptyMotor extends Motor {
  public EmptyMotor() {}

  @Override
  public void set(double output) {}

  @Override
  public Encoder setDistancePerPulse(double distancePerPulse) {
    return encoder;
  }

  @Override
  public double getDistance() {
    return 0;
  }

  @Override
  public double getRate() {
    return 0;
  }

  @Override
  public void resetEncoder() {}

  @Override
  public void stopAndResetEncoder() {}

  @Override
  public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {}

  @Override
  public int getCurrentPosition() {
    return 0;
  }

  @Override
  public double getCorrectedVelocity() {
    return 0;
  }

  @Override
  public void setRunMode(RunMode runmode) {}

  @Override
  protected double getVelocity() {
    return 0;
  }

  @Override
  public double get() {
    return 0;
  }

  @Override
  public void setTargetDistance(double target) {}

  @Override
  public void setInverted(boolean isInverted) {}

  @Override
  public boolean getInverted() {
    return false;
  }

  /** Disable the motor. */
  @Override
  public void disable() {}

  @Override
  public String getDeviceType() {
    return "Motor from null";
  }

  @Override
  public void stopMotor() {}
}
