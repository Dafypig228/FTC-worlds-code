package org.firstinspires.ftc.teamcode.controlllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Main;
import org.firstinspires.ftc.teamcode.Utilities.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import java.util.function.Supplier;

@Config
public class Outtake {
    public DcMotorEx leftFlyWheel, rightFlyWheel, transfer, turretEncoder;
    public Servo tangage, gate, turret, RighTturret;
    private double tangagePosition = 0.48, turretEncoderZero;

    public static double ticksPerVolt = 201.5625, CloseGatePos= 0.1, OpenGatePos = 0.25, minAngle = 0.275;
    private double kP = 0.004;
    public VoltageSensor sensor;
    public static double FarTan = 0.6, CloseTan = 0.85;
    private double targetVelocity = 0;
    private double goalX = 138, goalY = 144 - 6;
    private double tangageAngle = 1, FlyWheelVel = 2400;
    private double myX = 0, myY = 0, vH_robot = 0, vX_field = 0, vY_field = 0, realDistance = 0, iterVel = 0, iterAngle = 0;
    public static double goalYRed = -66, goalYBlue = 66;
    Intake intake;

    public Outtake(boolean Auto, HardwareMap hardwareMap) {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightFlyWheel");
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretEncoder = hardwareMap.get(DcMotorEx.class, "transfer");

        gate = hardwareMap.get(Servo.class, "gate");

        tangage = hardwareMap.get(Servo.class, "tangash");

        turret = hardwareMap.get(Servo.class, "turret");
        RighTturret = hardwareMap.get(Servo.class, "RightTurret");

        sensor = hardwareMap.voltageSensor.iterator().next();

        intake = new Intake(hardwareMap);
    }

    public Action openGate() {
        return new OpenGate();
    }

    public Action closeGate() {
        return new CloseGate();
    }

    public Action startTransfer() {
        return new StartTransfer();
    }

    public Action startTransferSlow() {
        return new StartTransferSlow();
    }

    public Action stopTransfer() {
        return new StopTransfer();
    }

    public Action setTangagePosition(double position) {
        return new SetTangagePosition(position);
    }

    public Action setFlyWheelVelocity(double targetV) {
        return new SetFlyWheelVelocity(targetV);
    }

    public Action VirtualGoalCalculateAuto(Supplier<Pose2d> poseEstiminate, Supplier<PinpointLocalizer> localizer, boolean RedAlliance, Supplier<Boolean> shootState) {
        return new VirtualGoalCalculateAuto(poseEstiminate, localizer, RedAlliance, shootState);
    }

    public Action flyWheelPIDF() {
        return new FlyWheelPIDF();
    }

    public double getVelocity() {
        return rightFlyWheel.getVelocity();
    }

    public class SetFlyWheelVelocity implements Action {
        double targetV;

        public SetFlyWheelVelocity(double targetV) {
            this.targetV = targetV;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            targetVelocity = this.targetV;
            return false;
        }
    }

    public void SetFlyWheelVelocity(double targetV) {
        targetVelocity = targetV;
    }

    public class FlyWheelPIDF implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double voltage = sensor.getVoltage();
            double error = targetVelocity - getVelocity();
            double kF = targetVelocity / ticksPerVolt / voltage;
            double correction = kP * error + kF;
            if (targetVelocity == 0) correction = 0;
            //double compensatedPower = correction * (nominalVoltage / voltage);
            leftFlyWheel.setPower(correction);
            rightFlyWheel.setPower(correction);
            return true;
        }
    }

    public void UpdateFlyWheelPIDF() {
        double voltage = sensor.getVoltage();
        double error = targetVelocity - getVelocity();
        double kF = targetVelocity / ticksPerVolt / voltage;
        double correction = kP * error + kF;
        if (targetVelocity == 0) correction = 0;
        leftFlyWheel.setPower(correction);
        rightFlyWheel.setPower(correction);

    }

    public class SetTangagePosition implements Action {
        public SetTangagePosition(double position) {
            if (position > 1) {
                position = 1;
            } else if (position < 0) {
                position = 0;
            }
            tangagePosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            tangage.setPosition(tangagePosition);
            return false;
        }
    }

    public class OpenGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gate.setPosition(OpenGatePos);
            return false;
        }
    }

    public void OpenGate() {
        gate.setPosition(OpenGatePos);
    }

    public class CloseGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gate.setPosition(CloseGatePos);
            return false;
        }
    }

    public void CloseGate() {
        gate.setPosition(CloseGatePos);
    }

    public class StartTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transfer.setPower(1);
            return false;
        }
    }

    public void StartTransfer() {
        transfer.setPower(1);
    }

    public class StartTransferSlow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transfer.setPower(0.8);
            return false;
        }
    }

    public void StartTransferSlow() {
        transfer.setPower(0.5);
    }

    public class StopTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transfer.setPower(0);
            return false;
        }
    }

    public void StopTransfer() {
        transfer.setPower(0);
    }

    public void SetTangageAngle(double Angle) {
        double AngleToPosition = Angle * -0.05 + 2.6;
        AngleToPosition = 1 - AngleToPosition;
        AngleToPosition = Math.max(AngleToPosition, minAngle);
        AngleToPosition = Math.min(1, AngleToPosition);
        tangage.setPosition(AngleToPosition);
        tangagePosition = AngleToPosition;
    }

    public class VirtualGoalCalculateAuto implements Action {
        Supplier<Boolean> shootSupplier;
        boolean ToShoot = false;
        Supplier<Pose2d> poseEstimateSup;
        Supplier<PinpointLocalizer> localizerSup;
        boolean allianceRed;

        public VirtualGoalCalculateAuto(Supplier<Pose2d> poseEstimate, Supplier<PinpointLocalizer> localizer, boolean allianceRed, Supplier<Boolean> shootState) {
            poseEstimateSup = poseEstimate;
            localizerSup = localizer;
            this.allianceRed = allianceRed;
            shootSupplier = shootState;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d poseEstimate = poseEstimateSup.get();
            PinpointLocalizer localizer = localizerSup.get();
            ToShoot = shootSupplier.get();

            vH_robot = localizer.driver.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            vX_field = localizer.driver.getVelX(DistanceUnit.INCH);
            vY_field = localizer.driver.getVelY(DistanceUnit.INCH);
            goalY = allianceRed ? goalYRed : goalYBlue;
            realDistance = Math.hypot(goalX - myX, goalY - myY);

            myX = poseEstimate.position.x;
            myY = poseEstimate.position.y;

            realDistance = Math.hypot(goalX - myX, goalY - myY);
            FlyWheelVel = 6.75 * realDistance + 666.87;
            tangageAngle = Math.toDegrees(Math.atan((2 * Main.h) / (realDistance - Main.peakDistance)));

            SetTangageAngle(tangageAngle);
            if (ToShoot) {
                SetFlyWheelVelocity(FlyWheelVel);

                if (getVelocity() >= FlyWheelVel - Main.FW_error && getVelocity() <= FlyWheelVel + Main.FW_error) {
                    transfer.setPower(1);
                    intake.intakeVoid();
                    OpenGate();
                }
            } else {
                SetFlyWheelVelocity(FlyWheelVel);
                CloseGate();
                StopTransfer();
            }

            return true;
        }
    }

}
