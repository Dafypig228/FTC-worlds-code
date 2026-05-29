package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Storage;
import org.firstinspires.ftc.teamcode.controlllers.Aim;
import org.firstinspires.ftc.teamcode.controlllers.Intake;
import org.firstinspires.ftc.teamcode.controlllers.Movement;
import org.firstinspires.ftc.teamcode.controlllers.Outtake;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import java.util.List;

@Config
@TeleOp
public class OuttakeTest extends LinearOpMode {
    Telemetry dashboardTelemetry;
    public static double tangagePos1 = 0, tangagePos2 = 1, gatePos1 = 0.55, gatePos2 = 0.4, flyWheelPower = 0.4, turretPos1 = 0, turretPos2 = 1;


    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Telemetry Setup
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.setMsTransmissionInterval(50);
        dashboardTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        MultipleTelemetry telemetryAll = new MultipleTelemetry(telemetry, dashboardTelemetry);
        telemetryAll.setMsTransmissionInterval(50);

        Servo gate = hardwareMap.get(Servo.class, "gate");

        Servo tangage = hardwareMap.get(Servo.class, "tangash");
            Servo turret = hardwareMap.get(Servo.class, "turret");
        Servo Rturret = hardwareMap.get(Servo.class, "RightTurret");

        DcMotorEx leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        //leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotorEx rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightFlyWheel");
        //rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            if (gamepad1.b) {
                turret.setPosition(turretPos2);
            }; //0.85
            if (gamepad1.a) {
                turret.setPosition(turretPos1);
            }; //0.1

            if (gamepad1.y) {
                Rturret.setPosition(turretPos2);
            }; //0.85
            if (gamepad1.x) {
                Rturret.setPosition(turretPos1);
            }; //0.1


            if (gamepad2.b) {gate.setPosition(gatePos1);};
            if (gamepad2.a) {gate.setPosition(gatePos2);};
            if (gamepad2.x) {tangage.setPosition(tangagePos1);};
            if (gamepad2.y) {tangage.setPosition(tangagePos2);};

            if (gamepad2.left_bumper) {
                leftFlyWheel.setPower(flyWheelPower);
                rightFlyWheel.setPower(flyWheelPower);
            }
            else {
                leftFlyWheel.setPower(0);
                rightFlyWheel.setPower(0);
            }

            if (gamepad2.left_trigger > 0.1) {leftFlyWheel.setPower(flyWheelPower);}
            else {leftFlyWheel.setPower(0);}
            if (gamepad2.right_trigger > 0.1) {rightFlyWheel.setPower(flyWheelPower);}
            else {rightFlyWheel.setPower(0);}

            telemetryAll.update();

        }
    }


}