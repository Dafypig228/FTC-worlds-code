package org.firstinspires.ftc.teamcode.controlllers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class RRPoint {
    private double x;
    private double y;
    private double headingDeg;

    public RRPoint(double x, double y, double headingDeg) {
        this.x = x;
        this.y = y;
        this.headingDeg = headingDeg;
    }


    public Pose2d toPose() {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    public Vector2d vec() {
        return new Vector2d(x, y);
    }

    public double rad() {
        return Math.toRadians(headingDeg);
    }
}