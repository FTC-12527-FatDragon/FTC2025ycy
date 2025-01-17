package org.firstinspires.ftc.teamcode.utils.RoadRunnerPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BooleanArea implements PoseArea{
    private final boolean ret;
    public BooleanArea(boolean toReturn){
        this.ret = toReturn;
    }
    public boolean covers(Pose2d pose){
        return ret;
    }
}
