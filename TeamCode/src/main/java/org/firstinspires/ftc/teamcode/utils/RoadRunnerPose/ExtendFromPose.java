package org.firstinspires.ftc.teamcode.utils.RoadRunnerPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode.LocalizationTest;

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
        Pose2d extendPose = pose.plus(new Pose2d(extendOffset.rotated(pose.getHeading()), 0));
        boolean covers = extendArea.covers(extendPose);
        TelemetryPacket packet = new TelemetryPacket();
        if(covers){
            packet.fieldOverlay().setStroke("#7FE1FF");
        }else packet.fieldOverlay().setStroke("#1A94BA");
        LocalizationTest.drawRobot(packet.fieldOverlay(), extendPose);
        packet.put("Extend Pose", extendPose);
        packet.put("Extend Covers", covers);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        return covers;
    }
}
