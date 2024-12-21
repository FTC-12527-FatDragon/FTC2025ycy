package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import lombok.Getter;

public class Pose2dHelperClass extends Translation2dHelperClass {
    @Getter
    public double Heading;

    public Pose2dHelperClass(double x, double y, double heading) {
        super(x,y);
        this.Heading = heading;
    }

    public Pose2dHelperClass() {
        this(0, 0, 0);
    }

    public Pose2d toPose2d(){
        return new Pose2d(X, Y, Heading);
    }
}
