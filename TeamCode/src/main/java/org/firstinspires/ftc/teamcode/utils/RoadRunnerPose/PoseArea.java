package org.firstinspires.ftc.teamcode.utils.RoadRunnerPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public interface PoseArea {
    boolean covers(Pose2d pose);
}
