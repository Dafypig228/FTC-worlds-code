package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;

@Config
public class Storage {
    public static Pose2d CurrentPose = new Pose2d(72, 0, 0);
    public static Pose CurrentPosePedro = new Pose(5.4, 0, 0);
    public static Pose FarPose = new Pose(5.4, 5.3, Math.toRadians(-90));
    public static Pose ClosePose = new Pose(132, -40, Math.toRadians(-90));
    public static double turretEncoderZero = 0;
    public static boolean allianceBlue = false;
}
