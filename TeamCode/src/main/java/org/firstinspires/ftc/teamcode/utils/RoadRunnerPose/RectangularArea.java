package org.firstinspires.ftc.teamcode.utils.RoadRunnerPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class RectangularArea implements PoseArea{
    final Vector2d center;
    final Vector2d translation;
    public RectangularArea(Vector2d center, double xLength, double yLength){
        this.center = center;
        translation = new Vector2d(xLength*0.5, yLength*0.5);
    }

    public boolean covers(Pose2d pose) {
        Vector2d v = pose.vec();
        if(Math.abs(v.getX()-center.getX())<=translation.getX() && Math.abs(v.getY()-center.getY())<=translation.getY())
        {
            return true;
        }
        return false;
    }

}
