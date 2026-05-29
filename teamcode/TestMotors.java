package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
public class TestMotors extends LinearOpMode {
    public VoltageSensor sensor;
    Outtake outtake;
    Intake intake;
    Movement robot;
    Telemetry dashboardTelemetry;
    ElapsedTime FWtimer;
    MecanumDrive drive;

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        outtake = new Outtake(false, hardwareMap);
        intake = new Intake(hardwareMap);


        waitForStart();


        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();


        }
    }


}