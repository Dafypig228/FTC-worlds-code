
package org.firstinspires.ftc.teamcode.pedroPathing;
// make sure this aligns with class location

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utilities.Storage;
import org.firstinspires.ftc.teamcode.controlllers.Aim;
import org.firstinspires.ftc.teamcode.controlllers.Intake;
import org.firstinspires.ftc.teamcode.controlllers.Outtake;

@Config
@Autonomous(name = "AutoFarFromGoalWithGate", group = "Competition")
public class AutoFarFromGoalWithGate extends OpMode {
    private boolean allianceBlue = false;
    private Intake intake;
    private Outtake outtake;
    MultipleTelemetry telemetryAll;
    private Follower follower;
    Aim aimer = new Aim();
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public static Vector2d goal = new Vector2d(138, -68);

    double ShootVel = 1600, RazgonVel = 900, FlyWheelVelocity = 1000, Static = 1650;

    private Pose startPose = new Pose(5.4, -13.8, Math.toRadians(-90));
    private Pose scorePose = new Pose(20, -17, Math.toRadians(-90));

    private Pose intakeHumanZonePose = new Pose(7, -65.75, Math.toRadians(-90));
    private Pose intakeGateZonePose = new Pose(5.5, -65.75, Math.toRadians(-90));
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
                actionTimer.resetTimer();
                outtake.closeGate();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if(actionTimer.getElapsedTimeSeconds() > 3) {
                    shoot();
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopShoot();
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                    intake();
                    follower.followPath(intakeHumanZone, 0.7, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    stopIntake();
                    follower.followPath(scoreHuman);
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.6) {
                    actionTimer.resetTimer();
                    shoot();
                    setPathState(8);
                }
                break;
            case 8:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopShoot();
                    intake();
                    follower.followPath(intakeGateZone, 0.7, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(scoreGate);
                    stopIntake();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                    shoot();
                    actionTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopShoot();
                    intake();
                    follower.followPath(intakeHumanZone, 0.7, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if (actionTimer.getElapsedTimeSeconds() > 1) {;
                    follower.followPath(scoreGate);
                    stopIntake();
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    shoot();
                    actionTimer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopShoot();
                    intake();
                    follower.followPath(intakeGateZone, 0.7, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    Storage.CurrentPosePedro = follower.getPose();
                    follower.followPath(scoreGate);
                    stopIntake();
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    Storage.CurrentPosePedro = follower.getPose();
                    setPathState(22);
                }
                break;
            case 22:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    shoot();
                    Storage.CurrentPosePedro = follower.getPose();
                    actionTimer.resetTimer();
                    setPathState(23);
                }
                break;
            case 23:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopShoot();
                    Storage.CurrentPosePedro = follower.getPose();
                    follower.followPath(leave);
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    Storage.CurrentPosePedro = follower.getPose();
                    stopShoot();
                    Static = 0;
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
        outtake.SetFlyWheelVelocity(Static);
        outtake.UpdateFlyWheelPIDF();
        autonomousPathUpdate();

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

    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            allianceBlue = !allianceBlue;
        }
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
        Storage.CurrentPosePedro = follower.getPose();
    }

    private void recalculatePoses() {
        goal = new Vector2d(goal.x, (allianceBlue? -goal.y : goal.y));
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
        outtake.StartTransferSlow();
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