package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilities.Storage.ClosePose;
import static org.firstinspires.ftc.teamcode.Utilities.Storage.CurrentPosePedro;
import static org.firstinspires.ftc.teamcode.Utilities.Storage.FarPose;

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
public class Main extends LinearOpMode {
    public static double FlyWheelVel = 1300, FlyWheelVelCustom = 400;
    public static double robotkP = 0.023, robotkI = 0.12, robotkD = 0.002, robotkF = -0.;
    public static double leadTimeCef = 480, h = 33.6, peakDistance = 5;
    public static double FW_error = 20;
    public VoltageSensor sensor;
    Outtake outtake;
    Intake intake;
    Movement robot;
    Telemetry dashboardTelemetry;
    ElapsedTime FWtimer;
    MecanumDrive drive;

    /// /////////////////////// ALLIANCE     /////////////////////////////////////////////
     boolean allianceRed = true;
    /// /////////////////////// ALLIANCE     /////////////////////////////////////////////
    boolean autoPricel = false;
    double tangageAngle = 45, turretEncoderZero = 0, heading = 0, turretPosition = 0;
    double leadTime, angleToLook;
    //	final int GOAL_Y = 66;
    double goalXCoor = 138, goalYCoor = 66;
    Aim aimer = new Aim();

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        outtake = new Outtake(false, hardwareMap);
        intake = new Intake(hardwareMap);

        FWtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // Telemetry Setup
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.setMsTransmissionInterval(50);
        dashboardTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        MultipleTelemetry telemetryAll = new MultipleTelemetry(telemetry, dashboardTelemetry);
        telemetryAll.setMsTransmissionInterval(50);
        allianceRed = true;

        Pose2d startPose = new Pose2d(
                CurrentPosePedro.getPose().getX(),
                CurrentPosePedro.getPose().getY(),
                allianceRed ? Math.toRadians(-90) : Math.toRadians(90)
        );

        drive = new MecanumDrive(hardwareMap, startPose);

        sensor = hardwareMap.voltageSensor.iterator().next();
        PinpointLocalizer localizer = (PinpointLocalizer) drive.localizer;
        localizer.driver.recalibrateIMU();
        drive.updatePoseEstimate();

        turretEncoderZero = Storage.turretEncoderZero;

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) {
                allianceRed = !allianceRed;
            }
            /*if (gamepad1.yWasPressed()){
                CurrentPosePedro = FarPose;
            } else { CurrentPosePedro = ClosePose;}*/
            telemetry.addData("ALLIANCE: ", !allianceRed ? "BLUE" : "RED");
            telemetry.update();
        }

        FWtimer.reset();

        waitForStart();

        robot = new Movement(
                hardwareMap, drive,
                robotkP, robotkI, robotkD, robotkF,
                360,
                1
        );

        outtake.UpdateFlyWheelPIDF();
        if (allianceRed) {
            goalYCoor = -goalYCoor;
        }

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            drive.updatePoseEstimate();
            Pose2d poseEstimate = drive.localizer.getPose();
                Vector2d pos = new Vector2d(poseEstimate.position.x, poseEstimate.position.y);

            double vH_robot = localizer.driver.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            Vector2d v = new Vector2d(localizer.driver.getVelX(DistanceUnit.INCH), localizer.driver.getVelY(DistanceUnit.INCH));
            heading = drive.localizer.getPose().heading.toDouble();

            if (gamepad2.dpadLeftWasPressed()) goalXCoor += 2 * (allianceRed ? 1 : -1);
            if (gamepad2.dpadRightWasPressed()) goalXCoor -= 2 * (allianceRed ? 1 : -1);
            if (gamepad2.dpadUpWasPressed()) goalYCoor -= 2 * (allianceRed ? 1 : -1);
            if (gamepad2.dpadDownWasPressed()) goalYCoor += 2 * (allianceRed ? 1 : -1);

            Vector2d goal = new Vector2d(goalXCoor, goalYCoor);

            aimer.aimOld(goal, v, pos, false);
            FlyWheelVel = aimer.flyWheelVel;
            tangageAngle = aimer.tangageAngle;

            double targetAngleRad = Math.atan2(aimer.aim_pos.y - pos.y, aimer.aim_pos.x - pos.x);
            angleToLook = Math.toDegrees(targetAngleRad);

            aimer.turretAimServo(angleToLook,Math.toDegrees(heading),  telemetryAll);
            turretPosition = aimer.turretPosition;
            //outtake.turret.setPosition(turretPosition);
            //outtake.RighTturret.setPosition(turretPosition);

            if (gamepad1.right_bumper) {
                intake.OuttakeStart();
            } else if (gamepad1.left_trigger > 0.3) {
                intake.intakeVoid();
                outtake.StartTransferSlow();
            } else if (gamepad1.right_trigger <= 0.5) {
                intake.Stop();
                outtake.StopTransfer();
            }
            if (gamepad1.right_trigger > 0.5) {
                FWtimer.reset();
            }

            if (gamepad1.right_trigger > 0.5 || FWtimer.milliseconds() < 200) {
                outtake.SetFlyWheelVelocity(0);
                autoPricel = false;

                double currentVel = outtake.getVelocity();
                double targetShootingVel = FlyWheelVel;

                if (currentVel >= targetShootingVel - FW_error && currentVel <= targetShootingVel + FW_error) {
                    outtake.StartTransfer();
                    intake.intakeVoid();
                    outtake.OpenGate();
                }
            } else {
                outtake.SetFlyWheelVelocity(0);
                outtake.CloseGate();
                autoPricel = false;
            }

            //outtake.SetTangageAngle(aimer.tangageAngle);

            robot.FieldCentricDrive(
                    (allianceRed ? -1.0 : 1.0) * gamepad1.left_stick_y,
                    (allianceRed ? -1.0 : 1.0) * gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    autoPricel,
                    angleToLook,
                    poseEstimate,
                    vH_robot
            );

            outtake.UpdateFlyWheelPIDF();

            // Telemetry
            telemetryAll.addLine("--- Coordinates");
            telemetryAll.addData("Goal X", goal.x);
            telemetryAll.addData("Goal Y", goal.y);
            telemetryAll.addData("Target X", aimer.aim_pos.x);
            telemetryAll.addData("Target Y", aimer.aim_pos.y);
            telemetryAll.addData("My X", pos.x);
            telemetryAll.addData("My Y", pos.y);
            telemetryAll.addData("Heading", Math.toDegrees(heading));

            telemetryAll.addLine("--- Ballistics");
            telemetryAll.addData("Lead Time", leadTime);
            telemetryAll.addData("Distance", aimer.virtualDistance);

            telemetryAll.addLine("--- Shooter");
            telemetryAll.addData("Target Vel", aimer.flyWheelVel);
            telemetryAll.addData("Actual Vel", outtake.getVelocity());
            telemetryAll.addData("Final Angle", aimer.tangageAngle);

            telemetryAll.addData("Alliance Red", allianceRed);
            telemetryAll.update();

//            b = false;
        }
    }


}