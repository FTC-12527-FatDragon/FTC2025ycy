package org.firstinspires.ftc.teamcode.utils.RoadRunnerPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CricularArea implements PoseArea{
    private final Vector2d center;
    private final double radius;
    public CricularArea(Vector2d center, double radius){
        this.center = center;
        this.radius = radius;
    }
    public boolean covers(Pose2d pose){
        return pose.vec().distTo(center)<=radius;
    }
}
