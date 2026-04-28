package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose currentPose = new Pose(0, 0, 0);

    public static void savePose(Pose pose) {
        currentPose = pose;
    }

    public static Pose getSavedPose() {
        return currentPose;
    }

    public static void reset() {
        currentPose = new Pose(0, 0, 0);
    }
}