
package org.firstinspires.ftc.teamcode.pedroPathing;
// make sure this aligns with class location

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utilities.Storage;
import org.firstinspires.ftc.teamcode.controlllers.Aim;
import org.firstinspires.ftc.teamcode.controlllers.Intake;
import org.firstinspires.ftc.teamcode.controlllers.Outtake;

@Config
@Autonomous(name = "Camera", group = "Competition")
public class Camera extends OpMode {
    private boolean allianceBlue;
    private Intake intake;
    private Limelight3A limelight;
    double ballFieldX;
    private Outtake outtake;
    MultipleTelemetry telemetryAll;
    private Follower follower;
    Aim aimer = new Aim();
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public static Vector2d goal = new Vector2d(147, -75);

    double ShootVel = 1200, RazgonVel = 900, FlyWheelVelocity = 1000;

    private Pose startPose = new Pose(5.4, -5.3, Math.toRadians(-90));
    private Pose scorePose = new Pose(20, -9, Math.toRadians(-90));

    private Pose intakeHumanZonePose = new Pose(7, -58.1, Math.toRadians(-90));
    private Pose intakeGateZonePose = new Pose(5.5, -58.1, Math.toRadians(-90));
    private Pose leavePose = new Pose(27,-30, Math.toRadians(-90));

    private Path scoreHuman, scoreGate, leave;
    private PathChain intakeHumanZone, intakeGateZone;

    public void buildPaths() {
        scoreHuman = new Path(new BezierLine(intakeHumanZonePose, scorePose));
        scoreHuman.setLinearHeadingInterpolation(intakeHumanZonePose.getHeading(), scorePose.getHeading());

        intakeHumanZone = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intakeHumanZonePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakeHumanZonePose.getHeading())
                .build();

        intakeGateZone = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeGateZonePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeGateZonePose.getHeading())
                .build();

        scoreGate = new Path(new BezierLine(intakeGateZonePose, scorePose));
        scoreGate.setLinearHeadingInterpolation(intakeGateZonePose.getHeading(), scorePose.getHeading());

        leave = new Path(new BezierLine(scorePose, leavePose));
        leave.setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.pathBuilder()
                        .addPath(new BezierLine(startPose, new Pose(ballFieldX, -58.1, Math.toRadians(-90))))
                        .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(-90))
                        .build();
            case 9:
                if (actionTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(intakeGateZone, 0.7, true);
                    setPathState(9);
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    Storage.CurrentPosePedro = follower.getPose();
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        Vector2d pos = new Vector2d(follower.getPose().getX(), follower.getPose().getY());
        Vector2d v = new Vector2d(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());
        aimer.aimOld(goal, v, pos, true);
        RazgonVel = aimer.flyWheelVel - 300;
        ShootVel = aimer.flyWheelVel;

        double targetAngleRad = Math.atan2(aimer.aim_pos.y - pos.y, aimer.aim_pos.x - pos.x);
        double angleToLook = Math.toDegrees(targetAngleRad);

        aimer.turretAimServo(angleToLook, Math.toDegrees(follower.getHeading()), telemetryAll);
        double turretPosition = aimer.turretPosition;
        outtake.turret.setPosition(turretPosition);
        outtake.RighTturret.setPosition(turretPosition);

        outtake.SetTangageAngle(aimer.tangageAngle);

        follower.update();
        outtake.SetFlyWheelVelocity(FlyWheelVelocity);
        outtake.UpdateFlyWheelPIDF();
        autonomousPathUpdate();


        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] py = result.getPythonOutput();
            if (py != null && py.length >= 8 && py[0] == 1) {
                double ballLateralIn = py[2] / 10.0;

                if (ballLateralIn > -7.5) ballFieldX = startPose.getX();
                else if (ballLateralIn < -7.5) ballFieldX = 12.5;
                telemetry.addData("ballFieldX", ballFieldX);

            }
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(true, hardwareMap);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        telemetryAll = new MultipleTelemetry(telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    @Override
    public void init_loop() {
        double ballLateralIn = 0;
        if (gamepad1.xWasPressed()) {
            allianceBlue = !allianceBlue;
        }
        telemetry.addData("ballFieldX", ballFieldX);
        telemetry.addData("ALLIANCE:", allianceBlue ? "BLUE" : "RED");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        recalculatePoses();

        outtake.UpdateFlyWheelPIDF();

        follower.setStartingPose(startPose);
        buildPaths();

        setPathState(0);
    }

    @Override
    public void stop() {
    }

    private void recalculatePoses() {
        goal = new Vector2d(goal.x, goal.y * (allianceBlue? -1: 1));
        startPose = new Pose(startPose.getX(), (allianceBlue ? -1 : 1) * startPose.getY(), startPose.getHeading() * (allianceBlue ? -1 : 1));
        scorePose = new Pose(scorePose.getX(), (allianceBlue ? -1 : 1) * scorePose.getY(), scorePose.getHeading() * (allianceBlue ? -1 : 1));
        leavePose = new Pose(leavePose.getX(), (allianceBlue ? -1 : 1) * leavePose.getY(), leavePose.getHeading() * (allianceBlue ? -1 : 1));
        intakeGateZonePose = new Pose(intakeGateZonePose.getX(), (allianceBlue ? -1 : 1) * intakeGateZonePose.getY(), intakeGateZonePose.getHeading() * (allianceBlue ? -1 : 1));
        intakeHumanZonePose = new Pose(intakeHumanZonePose.getX(), (allianceBlue ? -1 : 1) * intakeHumanZonePose.getY(), intakeHumanZonePose.getHeading() * (allianceBlue ? -1 : 1));
    }

    private void shoot() {
        intake.intakeVoid();
        outtake.StartTransfer();
        outtake.OpenGate();
        FlyWheelVelocity = ShootVel;
    }
    private void intake() {
        intake.intakeVoid();
        outtake.StartTransfer();
    }
    private void stopIntake() {
        intake.Stop();
        outtake.StopTransfer();
    }

    private void stopShoot() {
        intake.Stop();
        outtake.StopTransfer();
        outtake.CloseGate();
        FlyWheelVelocity = RazgonVel;
    }

    ;
}