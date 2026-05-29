
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
@Autonomous(name = "AutoCloseToGoalWithGate512", group = "Competition")
public class AutoCloseToGoalWithGateWithoutTimer extends OpMode {
    private boolean allianceBlue = false;
    private Intake intake;
    private Outtake outtake;
    MultipleTelemetry telemetryAll;
    private Follower follower;
    Aim aimer = new Aim();
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public static Vector2d goal = new Vector2d(138, -68);

    double ShootVel = 1200, RazgonVel = 900, FlyWheelVelocity = 1000, staticVel = 1275;

    private Pose startPose = new Pose(132, -40, Math.toRadians(-90));
    private Pose scorePose = new Pose(83, -12.7, Math.toRadians(-90));

    private Pose intakeFirstLinePose = new Pose(83, -59, Math.toRadians(-90));

    private Pose intakeSecondLinePose1 = new Pose(58, 2, Math.toRadians(-90));
    private Pose intakeSecondLinePose2 = new Pose(54, -63, Math.toRadians(-90));

    private Pose intakeThirdLinePose1 = new Pose(36, 8, Math.toRadians(-90));
    private Pose intakeThirdLinePose2 = new Pose(30, -63, Math.toRadians(-90));


    private Pose openGatePose1 = new Pose(60, -30, Math.toRadians(-90));
    private Pose openGatePose2 = new Pose(60, -56, Math.toRadians(-90));

    private Pose intakeGatePose = new Pose(48, -62, Math.toRadians(-20));

    private Pose leavePose = new Pose(72,-36, Math.toRadians(-90));

    private Path scorePreload, intakeFirstLine, scoreFirstLine, scoreSecondLine, preOpenGate, openGate, intakeGate, scoreGate, scoreThirdLine, leave;
    private PathChain intakeSecondLine, intakeThirdLine;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        scoreSecondLine = new Path(new BezierLine(intakeSecondLinePose2, scorePose));
        scoreSecondLine.setLinearHeadingInterpolation(intakeSecondLinePose2.getHeading(), scorePose.getHeading());

        intakeSecondLine = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intakeSecondLinePose1, intakeSecondLinePose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeSecondLinePose2.getHeading())
                .build();

        preOpenGate = new Path(new BezierLine(scorePose, openGatePose1));
        preOpenGate.setLinearHeadingInterpolation(scorePose.getHeading(), openGatePose1.getHeading());

        openGate = new Path(new BezierLine(openGatePose1, openGatePose2));
        openGate.setLinearHeadingInterpolation(openGatePose1.getHeading(), openGatePose2.getHeading());

        intakeGate = new Path(new BezierLine(openGatePose2, intakeGatePose));
        intakeGate.setLinearHeadingInterpolation(openGatePose2.getHeading(), intakeGatePose.getHeading());

        scoreGate = new Path(new BezierCurve(intakeGatePose, intakeSecondLinePose1, scorePose));
        scoreGate.setLinearHeadingInterpolation(intakeGatePose.getHeading(), intakeSecondLinePose1.getHeading(), scorePose.getHeading());

        intakeFirstLine = new Path(new BezierLine(scorePose, intakeFirstLinePose));
        intakeFirstLine.setLinearHeadingInterpolation(scorePose.getHeading(), intakeFirstLinePose.getHeading());

        scoreFirstLine = new Path(new BezierLine(intakeFirstLinePose, scorePose));
        scoreFirstLine.setLinearHeadingInterpolation(intakeFirstLinePose.getHeading(), scorePose.getHeading());

        intakeThirdLine = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intakeThirdLinePose1, intakeThirdLinePose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(),intakeThirdLinePose1.getHeading(), intakeThirdLinePose2.getHeading())
                .build();

        scoreThirdLine = new Path(new BezierLine(intakeThirdLinePose2, scorePose));
        scoreThirdLine.setLinearHeadingInterpolation(intakeThirdLinePose2.getHeading(), scorePose.getHeading());

        leave = new Path(new BezierLine(scorePose, leavePose));
        leave.setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                actionTimer.resetTimer();
                outtake.closeGate();
                follower.followPath(scorePreload);
                setPathState(1);

                break;
            case 1:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.8) {
                    shoot();
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if(actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopShoot();
                    intake();
                    follower.followPath(intakeSecondLine, 0.65, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                if(actionTimer.getElapsedTimeSeconds() > 0) {
                    stopIntake();
                    follower.followPath(scoreSecondLine);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    shoot();
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()  && actionTimer.getElapsedTimeSeconds() > 0.8) {
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                stopShoot();
                setPathState(8);
                follower.followPath(preOpenGate);
                break;
            case 8:
                follower.followPath(openGate);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                follower.followPath(intakeGate);
                intake();
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                stopIntake();
                follower.followPath(scoreGate);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    shoot();
                    setPathState(14);
                    actionTimer.resetTimer();
                }
                break;
            case 14:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.8) {
                    Storage.CurrentPosePedro = follower.getPose();
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20:
                Storage.CurrentPosePedro = follower.getPose();
                stopShoot();
                intake();
                setPathState(21);
                follower.followPath(intakeFirstLine);
                break;
            case 21:
                if (!follower.isBusy()) {
                    Storage.CurrentPosePedro = follower.getPose();
                    actionTimer.resetTimer();
                    setPathState(22);
                }
                break;
            case 22:
                if(actionTimer.getElapsedTimeSeconds() > 0) {
                    Storage.CurrentPosePedro = follower.getPose();
                    stopIntake();
                    follower.followPath(scoreFirstLine);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    Storage.CurrentPosePedro = follower.getPose();
                    shoot();
                    setPathState(24);
                    actionTimer.resetTimer();
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    Storage.CurrentPosePedro = follower.getPose();
                    actionTimer.resetTimer();
                    setPathState(25);
                }
                break;
            case 25:
                if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                    Storage.CurrentPosePedro = follower.getPose();
                    stopShoot();
                    setPathState(26);
                    follower.followPath(leave);
                }
                break;
            case 26:
                if(!follower.isBusy()){
                setPathState(-1);
                Storage.CurrentPosePedro = follower.getPose();}
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
        outtake.SetFlyWheelVelocity(staticVel);
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
        goal = new Vector2d(goal.x, goal.y * (allianceBlue? -1: 1));
        startPose = new Pose(startPose.getX(), (allianceBlue ? -1 : 1) * startPose.getY(), startPose.getHeading() * (allianceBlue ? -1 : 1));
        scorePose = new Pose(scorePose.getX(), (allianceBlue ? -1 : 1) * scorePose.getY(), scorePose.getHeading() * (allianceBlue ? -1 : 1));
        intakeFirstLinePose = new Pose(intakeFirstLinePose.getX(), (allianceBlue ? -1 : 1) * intakeFirstLinePose.getY(), intakeFirstLinePose.getHeading()  * (allianceBlue ? -1 : 1));
        intakeSecondLinePose1 = new Pose(intakeSecondLinePose1.getX(), (allianceBlue ? -1 : 1) * intakeSecondLinePose1.getY(), intakeSecondLinePose1.getHeading()  * (allianceBlue ? -1 : 1));
        intakeSecondLinePose2 = new Pose(intakeSecondLinePose2.getX(), (allianceBlue ? -1 : 1) * intakeSecondLinePose2.getY(), intakeSecondLinePose2.getHeading()  * (allianceBlue ? -1 : 1));
        intakeThirdLinePose1 = new Pose(intakeThirdLinePose1.getX(), (allianceBlue ? -1 : 1) * intakeThirdLinePose1.getY(), intakeThirdLinePose1.getHeading()  * (allianceBlue ? -1 : 1));
        intakeThirdLinePose2 = new Pose(intakeThirdLinePose2.getX(), (allianceBlue ? -1 : 1) * intakeThirdLinePose2.getY(), intakeThirdLinePose2.getHeading()  * (allianceBlue ? -1 : 1));
        openGatePose1 = new Pose(openGatePose1.getX(), (allianceBlue ? -1 : 1) * openGatePose1.getY(), openGatePose1.getHeading() * (allianceBlue ? -1 : 1));
        openGatePose2 = new Pose(openGatePose2.getX(), (allianceBlue ? -1 : 1) * openGatePose2.getY(), openGatePose2.getHeading()  * (allianceBlue ? -1 : 1));
        intakeGatePose = new Pose(intakeGatePose.getX(), (allianceBlue ? -1 : 1) * intakeGatePose.getY(), intakeGatePose.getHeading() * (allianceBlue ? -1 : 1));
        leavePose = new Pose(leavePose.getX(), (allianceBlue ? -1 : 1) * leavePose.getY(), leavePose.getHeading() * (allianceBlue ? -1 : 1));
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