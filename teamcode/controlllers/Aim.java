package org.firstinspires.ftc.teamcode.controlllers;

import static java.lang.Math.cos;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
@Config
public class Aim {
    public static double turretKp = 0.3, RdtoLookCustom = 0;
    public static boolean CustomTurretAngle = false;
    public Vector2d aim_pos;
    public double flyWheelVel = 1300, tangageAngle = 45, targetAngle = 0, turretAimTo = 0, turretAngleRad = 0, virtualDistance = 0;
    public static double minDegree = -205, maxDegree =120;
    public static double servoAtZeroDeg = 0.5;
    public static double servoPerDegree = 0.75/301;
    public double TICKS_PER_REV = 8192.0;
    public static  double turretServoLeftPose = 0.85, turretServoRightPose = 0.2, turretPosition = (turretServoLeftPose+turretServoRightPose)/2, offset = 100;
    public static double LEAD_TIME_CEF = 480, h = 33.6, PEAK_DISTANCE = 5, INITIAL_LEAD_TIME = 0.6;
    double leadTime = INITIAL_LEAD_TIME;

    public void aimOld(Vector2d goal, Vector2d v, Vector2d pos, boolean auto) {
        leadTime = INITIAL_LEAD_TIME;

        double safeAngle, safeVel;

        for (int i = 0; i < 5; i++) {
            safeAngle = max(1, tangageAngle);
            safeVel = max(100, flyWheelVel);

            this.aim_pos = goal.minus(v.times(leadTime));

            leadTime = LEAD_TIME_CEF / (safeVel * cos(Math.toRadians(safeAngle)));

            targetAngle = Math.atan2(aim_pos.y - pos.y, aim_pos.x - pos.x);
            virtualDistance = Math.hypot(aim_pos.x - pos.x, aim_pos.y - pos.y);


            flyWheelVel = 6.75 * virtualDistance + 666.87;

            double distTerm = max(0.1, virtualDistance - PEAK_DISTANCE);
            tangageAngle = Math.toDegrees(Math.atan((2 * h) / distTerm));
        }
    }


    public void aim(Vector2d goal, Vector2d v, Vector2d pos) {
        leadTime = INITIAL_LEAD_TIME;

        this.aim_pos = goal.minus(v.times(leadTime));

        targetAngle = Math.atan2(aim_pos.y - pos.y, aim_pos.x - pos.x);
        virtualDistance = Math.hypot(aim_pos.x - pos.x, aim_pos.y - pos.y);

        flyWheelVel = 6.75 * virtualDistance + 666.87;

        double distTerm = max(0.1, virtualDistance - PEAK_DISTANCE);
        tangageAngle = Math.toDegrees(Math.atan((2 * h) / distTerm));
    }

    public void turretAim(double heading, double TurretEncoder, double turretEncoderZero, MultipleTelemetry telemetryAll){
        turretAimTo = normalizeAngleRad(targetAngle - heading);
        double encRealTick = TurretEncoder - turretEncoderZero;
        turretAngleRad = ((encRealTick / TICKS_PER_REV) * 2.0 * Math.PI / 4004 * 4489);

        double correction = (turretAimTo - turretAngleRad) * turretKp + turretAimTo;

        if (correction > 1.5) {
            correction = 1.5;
        }
        turretPosition = (turretServoLeftPose - turretServoRightPose) * (Math.toDegrees(correction) + 90) / 180 + turretServoRightPose;

        telemetryAll.addLine("--- Turret");
        telemetryAll.addData("Turret Target Angle", targetAngle);
        telemetryAll.addData("Turret Servo Pos", turretPosition);
        telemetryAll.addData("Turret Aim To", turretAimTo);
        telemetryAll.addData("encRealTick", encRealTick);
    }

    public void turretAimServo(double RdtoLook, double heading, MultipleTelemetry telemetryAll) {
        double angleToLook = normalizeAngle(CustomTurretAngle ? RdtoLookCustom : heading - RdtoLook);

        if (angleToLook > maxDegree) {
            double alt = angleToLook - 360;
            if (alt >= minDegree) angleToLook = alt;
        } else if (angleToLook < minDegree) {
            double alt = angleToLook + 360;
            if (alt <= maxDegree) angleToLook = alt;
        }

        angleToLook = Math.max(minDegree, Math.min(maxDegree, angleToLook));

        turretPosition = servoAtZeroDeg + angleToLook * servoPerDegree;

        turretPosition = Math.max(turretServoRightPose, Math.min(turretServoLeftPose, turretPosition));

        telemetryAll.addLine("--- Turret");
        telemetryAll.addData("Turret Target Angle", Math.toDegrees( targetAngle));
        telemetryAll.addData("angleToLook", RdtoLook);
        telemetryAll.addData("angleToLookDone", angleToLook);
        telemetryAll.addData("Servo Pos", turretPosition);
    }

    double normalizeAngleRad(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    private double normalizeAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        return angle - 180;
    }

}
