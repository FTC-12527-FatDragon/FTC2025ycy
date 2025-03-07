package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import lombok.Getter;

public class Pose2dHelperClass extends Translation2dHelperClass {
  @Getter public double HeadingDeg;

  public Pose2dHelperClass(double x, double y, double headingdeg) {
    super(x, y);
    this.HeadingDeg = headingdeg;
  }

  public Pose2dHelperClass(Translation2dHelperClass tran, double headingdeg) {
    super(tran);
    this.HeadingDeg = headingdeg;
  }

  public Pose2dHelperClass(Pose2d pose){
    super(pose.getX(), pose.getY());
    this.HeadingDeg = Math.toDegrees(pose.getHeading());
  }

  public Pose2dHelperClass() {
    this(0, 0, 0);
  }

  public Pose2d toPose2d() {
    return new Pose2d(X, Y, getHeadingRad());
  }

  public double getHeadingRad() {
    return Math.toRadians(HeadingDeg);
  }
}
