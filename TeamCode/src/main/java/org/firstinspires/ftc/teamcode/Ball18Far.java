package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Ball18Far", group = "Autonomous")
@Configurable
public class Ball18Far extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo blocker;
    private TurretController turretController;

    // --- Configurable Timings & Positions ---
    public double shootVelocity = 1500;
    public double spinUpTime = 0.8;
    public double blockerOpen = 0.3;
    public double blockerClosed = 1.0;
    public double fullDumpDuration = 1.2;
    public double blockerMoveDelay = 0.25;

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    private Pose lastPose = new Pose(0, 0, 0);
    private long lastUpdateTime = 0;
    private Pose currentVelocity = new Pose(0, 0, 0);
    private boolean isShooting = false;

    public enum PathState {
        WAIT_FOR_HOMING,
        SHOOT_PRELOAD_SPINUP, SHOOT_PRELOAD,
        PREP_INTAKE_HP, DRIVE_TO_INTAKE_HP, DRIVE_TO_GO_OUT, DRIVE_BACK_IN, DRIVE_TO_SHOOT_2, SHOOT_2,
        PREP_SPIKE_3, DRIVE_TO_SPIKE_3, DRIVE_TO_SHOOT_3, SHOOT_3,
        PREP_CHANCE_1, DRIVE_TO_CHANCE_1, DRIVE_TO_SHOOT_4, SHOOT_4,
        PREP_CHANCE_2, DRIVE_TO_CHANCE_2, DRIVE_TO_SHOOT_5, SHOOT_5,
        PREP_CHANCE_3, DRIVE_TO_CHANCE_3, DRIVE_TO_SHOOT_6, SHOOT_6,
        PREP_LEAVE, DRIVE_TO_LEAVE, DONE
    }

    private PathState pathState;

    // --- Poses ---
    private final Pose startPose    = new Pose(54.1, 7.2,  Math.toRadians(180));
    private final Pose intakeHP     = new Pose(7.8,  8.3,  Math.toRadians(180));
    private final Pose goOutPose    = new Pose(15.6, 8.6,  Math.toRadians(180));
    private final Pose shootPose    = new Pose(54.1, 8.0,  Math.toRadians(180));
    private final Pose spike3Pose   = new Pose(9.9,  35.0, Math.toRadians(180));
    private final Pose chancePose   = new Pose(7.7,  44.5, Math.toRadians(90));
    private final Pose leavePose    = new Pose(36.2, 10.3, Math.toRadians(180));

    private final Pose cpSpike3    = new Pose(69.9, 39.7, Math.toRadians(180));
    private final Pose cpChance1   = new Pose(4.1, 17.5, Math.toRadians(180));
    private final Pose cpChance2   = new Pose(22.8, 12.7, Math.toRadians(180));

    private PathChain p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13;

    public void buildPaths() {
        p1 = follower.pathBuilder().addPath(new BezierLine(startPose, intakeHP)).setLinearHeadingInterpolation(startPose.getHeading(), intakeHP.getHeading()).build();
        p2 = follower.pathBuilder().addPath(new BezierLine(intakeHP, goOutPose)).setConstantHeadingInterpolation(intakeHP.getHeading()).build();
        p3 = follower.pathBuilder().addPath(new BezierLine(goOutPose, intakeHP)).setConstantHeadingInterpolation(intakeHP.getHeading()).build();
        p4 = follower.pathBuilder().addPath(new BezierLine(intakeHP, shootPose)).setConstantHeadingInterpolation(shootPose.getHeading()).build();
        p5 = follower.pathBuilder().addPath(new BezierCurve(shootPose, cpSpike3, spike3Pose)).setConstantHeadingInterpolation(spike3Pose.getHeading()).build();
        p6 = follower.pathBuilder().addPath(new BezierLine(spike3Pose, shootPose)).setConstantHeadingInterpolation(shootPose.getHeading()).build();
        p7 = follower.pathBuilder().addPath(new BezierCurve(shootPose, cpChance1, cpChance2, chancePose)).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();
        p8 = follower.pathBuilder().addPath(new BezierLine(chancePose, shootPose)).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();
        p9 = follower.pathBuilder().addPath(new BezierCurve(shootPose, cpChance1, cpChance2, chancePose)).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();
        p10 = follower.pathBuilder().addPath(new BezierLine(chancePose, shootPose)).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();
        p11 = follower.pathBuilder().addPath(new BezierCurve(shootPose, cpChance1, cpChance2, chancePose)).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();
        p12 = follower.pathBuilder().addPath(new BezierLine(chancePose, shootPose)).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();
        p13 = follower.pathBuilder().addPath(new BezierLine(shootPose, leavePose)).setConstantHeadingInterpolation(leavePose.getHeading()).build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        isShooting = false;
    }

    private void runShootingLogic(PathState nextState) {
        if (!isShooting) {
            blocker.setPosition(blockerOpen);
            isShooting = true;
            kickTimer.resetTimer();
        } else {
            // Wait slightly before slamming the intake on so the blocker can move
            if (kickTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                intake.setPower(1.0);
            }
            if (kickTimer.getElapsedTimeSeconds() > fullDumpDuration) {
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                setPathState(nextState);
            }
        }
    }

    public void statePathUpdate() {
        follower.setMaxPower(0.9);

        switch (pathState) {
            case WAIT_FOR_HOMING:
                if (turretController.currentState == TurretController.TurretState.TRACKING || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    setPathState(PathState.SHOOT_PRELOAD_SPINUP);
                }
                break;

            case SHOOT_PRELOAD_SPINUP:
                if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                runShootingLogic(PathState.PREP_INTAKE_HP);
                break;

            case PREP_INTAKE_HP:
                blocker.setPosition(blockerClosed);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p1);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_INTAKE_HP);
                }
                break;

            case DRIVE_TO_INTAKE_HP:
                if (!follower.isBusy()) {
                    follower.followPath(p2);
                    setPathState(PathState.DRIVE_TO_GO_OUT);
                }
                break;

            case DRIVE_TO_GO_OUT:
                if (!follower.isBusy()) {
                    follower.followPath(p3);
                    setPathState(PathState.DRIVE_BACK_IN);
                }
                break;

            case DRIVE_BACK_IN:
                if (!follower.isBusy()) {
                    follower.followPath(p4, true); // Hold end for shooting
                    intake.setPower(0); // Optional: turn off to prevent jamming before arrive
                    setPathState(PathState.DRIVE_TO_SHOOT_2);
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) setPathState(PathState.SHOOT_2);
                } else { pathTimer.resetTimer(); }
                break;

            case SHOOT_2:
                runShootingLogic(PathState.PREP_SPIKE_3);
                break;

            case PREP_SPIKE_3:
                blocker.setPosition(blockerClosed);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p5);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_SPIKE_3);
                }
                break;

            case DRIVE_TO_SPIKE_3:
                if (!follower.isBusy()) {
                    follower.followPath(p6, true);
                    intake.setPower(0);
                    setPathState(PathState.DRIVE_TO_SHOOT_3);
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) setPathState(PathState.SHOOT_3);
                } else { pathTimer.resetTimer(); }
                break;

            case SHOOT_3:
                runShootingLogic(PathState.PREP_CHANCE_1);
                break;

            case PREP_CHANCE_1:
                blocker.setPosition(blockerClosed);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p7);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_CHANCE_1);
                }
                break;

            case DRIVE_TO_CHANCE_1:
                if (!follower.isBusy()) {
                    follower.followPath(p8, true);
                    intake.setPower(0);
                    setPathState(PathState.DRIVE_TO_SHOOT_4);
                }
                break;

            case DRIVE_TO_SHOOT_4:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) setPathState(PathState.SHOOT_4);
                } else { pathTimer.resetTimer(); }
                break;

            case SHOOT_4:
                runShootingLogic(PathState.PREP_CHANCE_2);
                break;

            case PREP_CHANCE_2:
                blocker.setPosition(blockerClosed);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p9);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_CHANCE_2);
                }
                break;

            case DRIVE_TO_CHANCE_2:
                if (!follower.isBusy()) {
                    follower.followPath(p10, true);
                    intake.setPower(0);
                    setPathState(PathState.DRIVE_TO_SHOOT_5);
                }
                break;

            case DRIVE_TO_SHOOT_5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) setPathState(PathState.SHOOT_5);
                } else { pathTimer.resetTimer(); }
                break;

            case SHOOT_5:
                runShootingLogic(PathState.PREP_CHANCE_3);
                break;

            case PREP_CHANCE_3:
                blocker.setPosition(blockerClosed);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p11);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_CHANCE_3);
                }
                break;

            case DRIVE_TO_CHANCE_3:
                if (!follower.isBusy()) {
                    follower.followPath(p12, true);
                    intake.setPower(0);
                    setPathState(PathState.DRIVE_TO_SHOOT_6);
                }
                break;

            case DRIVE_TO_SHOOT_6:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) setPathState(PathState.SHOOT_6);
                } else { pathTimer.resetTimer(); }
                break;

            case SHOOT_6:
                runShootingLogic(PathState.PREP_LEAVE);
                break;

            case PREP_LEAVE:
                blocker.setPosition(blockerClosed);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p13);
                    setPathState(PathState.DRIVE_TO_LEAVE);
                }
                break;

            case DRIVE_TO_LEAVE:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                stopMechanisms();
                break;
        }
    }

    private void stopMechanisms() {
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        intake.setPower(0);
        blocker.setPosition(blockerClosed);
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        kickTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        lastPose = startPose;

        shooter1 = hardwareMap.get(DcMotorEx.class, "RS");
        shooter2 = hardwareMap.get(DcMotorEx.class, "LS");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        turretController = new TurretController(hardwareMap, "Turret");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(90, 0, 0, 16);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        blocker.setPosition(blockerClosed);
        buildPaths();
        setPathState(PathState.WAIT_FOR_HOMING);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        follower.update();

        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;
        Pose currentPose = follower.getPose();

        if (dt > 0) {
            double vx = (currentPose.getX() - lastPose.getX()) / dt;
            double vy = (currentPose.getY() - lastPose.getY()) / dt;
            double vHeading = (currentPose.getHeading() - lastPose.getHeading()) / dt;
            currentVelocity = new Pose(vx, vy, vHeading);
        }

        lastPose = currentPose;
        lastUpdateTime = currentTime;

        if (pathState != PathState.DONE) {
            shooter1.setVelocity(shootVelocity);
            shooter2.setVelocity(shootVelocity);
        }

        statePathUpdate();
        turretController.aimAtGoalWithPrediction(currentPose, currentVelocity);

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Turret State", turretController.currentState);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        Pose finalPose = follower.getPose();
        PedroPose.saveCurrentPose(finalPose);

        if (turretController != null) {
            PedroPose.saveTurretTicks(turretController.getRawTicks());
        }

        super.stop();
    }
}