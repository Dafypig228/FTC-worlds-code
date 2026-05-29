package org.firstinspires.ftc.teamcode.controlllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
@Config
public class Movement {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    public static double maxSpeedChangePerSec = 5, maxRotateSpeedChangePerSec = 5, ShapeExpo = 1, ShapeExpoRotate = 1;

    // PID Config
    private double kP, kI, kD, kF;
    private double integral = 0;
    private double lastError = 0;
    private double lastRotX = 0, lastRotY = 0, lastRotR = 0;

    // Rotate speed config (deg/sec per stick input)
    private double rotateSpeedDegPerSec;

    // Max drive power scaling
    private double drivePowerScale;

    // Heading target
    private double targetHeading = 0;

    // Loop timing
    private long lastTime = System.nanoTime();

    private MecanumDrive drive;
//	private Follower follower;

    private double correction = 0;

    public Movement(HardwareMap hardwareMap, MecanumDrive drive,
                    double kP, double kI, double kD, double kF,
                    double rotateSpeedDegPerSec,
                    double drivePowerScale) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        this.drive = drive;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.rotateSpeedDegPerSec = rotateSpeedDegPerSec;
        this.drivePowerScale = drivePowerScale;

        targetHeading = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
    }

//	public Movement(HardwareMap hardwareMap, Follower follower,
//					double kP, double kI, double kD, double kF,
//					double rotateSpeedDegPerSec,
//					double drivePowerScale) {
//
//		frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//		frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//		backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//		backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//
//		frontLeft.setDirection(DcMotor.Direction.REVERSE);
//		backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//		this.follower = follower;
//		this.kP = kP;
//		this.kI = kI;
//		this.kD = kD;
//		this.kF = kF;
//		this.rotateSpeedDegPerSec = rotateSpeedDegPerSec;
//		this.drivePowerScale = drivePowerScale;
//
//		targetHeading = Math.toDegrees(follower.getHeading());
//	}

    public void RobotCentricdrive(double x, double y, double rotateStick) {

        Pose2d poseEstimate = drive.localizer.getPose();
        double heading = Math.toDegrees(poseEstimate.heading.toDouble());
        drive.updatePoseEstimate();
        heading = normalizeAngle(heading);

        double deltaTime = getDeltaTimeSec();

        // Increment target heading
        targetHeading += rotateStick * rotateSpeedDegPerSec * deltaTime;
        targetHeading = normalizeAngle(targetHeading);

        // PID
        double error = normalizeAngle(heading - targetHeading);
        integral = integral * 0.9 + error * deltaTime;
        double derivative = normalizeAngle(error - lastError) * deltaTime;
        double correction = kP * error + kI * integral + kD * derivative + kF * rotateStick;
        lastError = error;

        double rx = correction;

        // double rx = -rotateStick;

        // Mecanum kinematics
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator * drivePowerScale;
        double backLeftPower = (y - x + rx) / denominator * drivePowerScale;
        double frontRightPower = (y - x - rx) / denominator * drivePowerScale;
        double backRightPower = (y + x - rx) / denominator * drivePowerScale;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void FieldCentricDrive(double x, double y, double rotateStick, boolean autoPricel, double angleToLook, Pose2d poseEstimateMain, double vH_robot) {
        double botHeadingRad = poseEstimateMain.heading.toDouble();

        double rotX = x * Math.cos(-botHeadingRad) - y * Math.sin(-botHeadingRad);
        double rotY = x * Math.sin(-botHeadingRad) + y * Math.cos(-botHeadingRad);

        double heading = Math.toDegrees(botHeadingRad);
        heading = normalizeAngle(heading);

        double deltaTime = getDeltaTimeSec();

        targetHeading += rotateStick * rotateSpeedDegPerSec * deltaTime;
        targetHeading = autoPricel ? normalizeAngle(angleToLook) : normalizeAngle(targetHeading);

        // PID calculation
        double error = normalizeAngle(heading - targetHeading);
        integral = (Math.abs(error) < 10) ? integral * 0.9 + error * deltaTime : 0;
        correction = kP * error + kI * integral + kD * vH_robot + kF * rotateStick;
        lastError = error;

        double rx = correction;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator * drivePowerScale;
        double backLeftPower = (rotY - rotX + rx) / denominator * drivePowerScale;
        double frontRightPower = (rotY - rotX - rx) / denominator * drivePowerScale;
        double backRightPower = (rotY + rotX - rx) / denominator * drivePowerScale;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }


    public double getHeading() {
        return Math.toDegrees(drive.localizer.getPose().heading.toDouble());
    }

    public void setTarget(double target) {
        targetHeading = target;
    }

    public void setDrivePowerScale(double scale) {
        this.drivePowerScale = scale;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    private double normalizeAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        return angle - 180;
    }

    private double getDeltaTimeSec() {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        return dt;
    }
    private double shapeInput(double stick, double expo) {
        double sign = Math.signum(stick);
        double scaled = Math.abs(stick);
        return sign * (expo * Math.pow(scaled, 3) + (1 - expo) * scaled);
    }
    private double slew(double target, double current, double maxChangePerSec, double dt) {
        double maxDelta = maxChangePerSec * dt;
        double delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }

}