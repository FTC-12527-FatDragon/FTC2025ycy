package org.firstinspires.ftc.teamcode.lib.pedropathingutil;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class PathChainDurationGetter {
    public static double getChainDuration(PathChain chain, Follower follower) {
        double res = 0.0;
        for (int i = 0; i < chain.size(); ++i) {
            res += chain.getPath(i).length() / follower.getVelocityMagnitude();
        }
        return res;
    }
}
