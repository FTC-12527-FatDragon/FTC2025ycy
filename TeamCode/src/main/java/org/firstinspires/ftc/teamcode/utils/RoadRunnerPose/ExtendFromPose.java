package org.firstinspires.ftc.teamcode.utils.RoadRunnerPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

// NOTE: It's a custom pose
public class ExtendFromPose implements PoseArea{
//    double extendLength;
//    double extendAngleFromHead;
    private final Vector2d extendOffset;
    private final PoseArea extendArea;

    /**
     * Calculates the extend position of a structure and see if it's in extendArea
     * @param extendArea The extend area.
     * @param extendOffset The offset, based on your point of interest, following the +x:forward and +y:left coordinate.
     */
    public ExtendFromPose(PoseArea extendArea, Vector2d extendOffset){
        this.extendArea = extendArea;
        this.extendOffset = extendOffset;
    }
    public boolean covers(Pose2d pose){
        return extendArea.covers(pose.plus(new Pose2d(extendOffset.rotated(pose.getHeading()), 0)));
    }
}
