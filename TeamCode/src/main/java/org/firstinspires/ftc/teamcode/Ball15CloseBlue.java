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

@Autonomous
@Configurable
public class Ball15CloseBlue extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo blocker;

    private TurretController turretController;
    public double shootVelocity = 1550;

    // --- Configurable Timings & Positions ---
    public double spinUpTime = 0;
    public double blockerOpen = 0.3;
    public double blockerClosed = 1.0;
    public double fullDumpDuration = 1;
    public double blockerMoveDelay = 0.25;
    public double intakeExtraTrafficTime = 0.5; // How long to run intake while driving back

    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    private Pose lastPose = new Pose(0,0,0);
    private long lastUpdateTime = 0;
    private Pose currentVelocity = new Pose(0,0,0);

    private boolean isShooting = false;

    public enum PathState {
        WAIT_FOR_HOMING,
        WAIT_INITIAL_ARRIVAL, SHOOT_PRELOAD,
        PREP_INTAKE_1, DRIVE_TO_SPIKE_2, DRIVE_TO_OPEN_GATE, DRIVE_TO_SHOOT_2, SHOOT_2,
        PREP_INTAKE_2, DRIVE_TO_GATE_1, WAIT_AT_GATE_1, DRIVE_TO_SHOOT_3, SHOOT_3,
        PREP_INTAKE_3, DRIVE_TO_GATE_2, WAIT_AT_GATE_2, DRIVE_TO_SHOOT_4, SHOOT_4,
        PREP_INTAKE_4, DRIVE_TO_SPIKE_1, DRIVE_TO_SHOOT_5, SHOOT_5,
        DONE
    }

    PathState pathState;

    // --- Poses ---
    private final Pose startPose      = new Pose(23.4, 125.2, Math.toRadians(135));
    private final Pose shootPreload   = new Pose(58.0, 82.7,  Math.toRadians(180));
    private final Pose controlshootPreloadintakeSpike2 = new Pose(66.1, 56.3, Math.toRadians(180));
    private final Pose intakeSpike2   = new Pose(12.4, 58.9,  Math.toRadians(180));
    private final Pose controlintakeSpike2openGatePose = new Pose(35.9, 64.6, Math.toRadians(180));
    private final Pose openGatePose   = new Pose(15.1, 68.4,  Math.toRadians(180));
    private final Pose intakeGatePose = new Pose(8.5,  57.5,  Math.toRadians(140));
    private final Pose intakeSpike1   = new Pose(15.2, 82.6,  Math.toRadians(180));
    private final Pose shoot5Pose     = new Pose(56.5, 105.7, Math.toRadians(180));
    private final Pose contolOpenGatePoseShootPreload = new Pose(61.3, 60.8, Math.toRadians(180));
    private final Pose controlShootPreloadIntakeGatePose = new Pose(61.3, 60.8, Math.toRadians(180));
    private final Pose controlIntakeGatePoseShootPreload = new Pose(50.9, 67.0, Math.toRadians(180));

    private PathChain p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;

    public void buildPaths() {
        p1 = follower.pathBuilder().addPath(new BezierLine(startPose, shootPreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreload.getHeading()).build();

        p2 = follower.pathBuilder().addPath(new BezierCurve(shootPreload, controlshootPreloadintakeSpike2, intakeSpike2))
                .setConstantHeadingInterpolation(intakeSpike2.getHeading()).build();

        p3 = follower.pathBuilder().addPath(new BezierCurve(intakeSpike2, controlintakeSpike2openGatePose, openGatePose))
                .setConstantHeadingInterpolation(openGatePose.getHeading()).build();

        p4 = follower.pathBuilder().addPath(new BezierCurve(openGatePose, contolOpenGatePoseShootPreload, shootPreload))
                .setConstantHeadingInterpolation(shootPreload.getHeading()).build();

        p5 = follower.pathBuilder().addPath(new BezierCurve(shootPreload, controlShootPreloadIntakeGatePose, intakeGatePose))
                .setLinearHeadingInterpolation(shootPreload.getHeading(), intakeGatePose.getHeading()).build();

        p6 = follower.pathBuilder().addPath(new BezierCurve(intakeGatePose, controlIntakeGatePoseShootPreload, shootPreload))
                .setLinearHeadingInterpolation(intakeGatePose.getHeading(), shootPreload.getHeading()).build();

        p7 = follower.pathBuilder().addPath(new BezierCurve(shootPreload,controlShootPreloadIntakeGatePose, intakeGatePose))
                .setLinearHeadingInterpolation(shootPreload.getHeading(), intakeGatePose.getHeading()).build();

        p8 = follower.pathBuilder().addPath(new BezierCurve(intakeGatePose, controlIntakeGatePoseShootPreload, shootPreload))
                .setLinearHeadingInterpolation(intakeGatePose.getHeading(), shootPreload.getHeading()).build();

        p9 = follower.pathBuilder().addPath(new BezierLine(shootPreload, intakeSpike1))
                .setConstantHeadingInterpolation(intakeSpike1.getHeading()).build();

        p10 = follower.pathBuilder().addPath(new BezierLine(intakeSpike1, shoot5Pose))
                .setConstantHeadingInterpolation(shoot5Pose.getHeading()).build();
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
                    follower.followPath(p1);
                    setPathState(PathState.WAIT_INITIAL_ARRIVAL);
                }
                break;

            case WAIT_INITIAL_ARRIVAL:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case SHOOT_PRELOAD:
                runShootingLogic(PathState.PREP_INTAKE_1);
                break;

            case PREP_INTAKE_1:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p2);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_SPIKE_2);
                }
                break;

            case DRIVE_TO_SPIKE_2:
                if (!follower.isBusy()) {
                    follower.followPath(p3);
                    setPathState(PathState.DRIVE_TO_OPEN_GATE);
                }
                break;

            case DRIVE_TO_OPEN_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(p4);
                    intake.setPower(0); // Standard off (no extra run here per request)
                    blocker.setPosition(blockerClosed);
                    setPathState(PathState.DRIVE_TO_SHOOT_2);
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.SHOOT_2);
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case SHOOT_2:
                runShootingLogic(PathState.PREP_INTAKE_2);
                break;

            case PREP_INTAKE_2:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p5);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_GATE_1);
                }
                break;

            case DRIVE_TO_GATE_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AT_GATE_1);
                }
                break;

            case WAIT_AT_GATE_1:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(p6);
                    intake.setPower(1.0); // Extra run
                    blocker.setPosition(blockerClosed);
                    setPathState(PathState.DRIVE_TO_SHOOT_3);
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (pathTimer.getElapsedTimeSeconds() > intakeExtraTrafficTime) {
                    intake.setPower(0);
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                runShootingLogic(PathState.PREP_INTAKE_3);
                break;

            case PREP_INTAKE_3:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p7);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_GATE_2);
                }
                break;

            case DRIVE_TO_GATE_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AT_GATE_2);
                }
                break;

            case WAIT_AT_GATE_2:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(p8);
                    intake.setPower(1.0); // Extra run
                    blocker.setPosition(blockerClosed);
                    setPathState(PathState.DRIVE_TO_SHOOT_4);
                }
                break;

            case DRIVE_TO_SHOOT_4:
                if (pathTimer.getElapsedTimeSeconds() > intakeExtraTrafficTime) {
                    intake.setPower(0);
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_4);
                }
                break;

            case SHOOT_4:
                runShootingLogic(PathState.PREP_INTAKE_4);
                break;

            case PREP_INTAKE_4:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    follower.followPath(p9);
                    intake.setPower(1.0);
                    setPathState(PathState.DRIVE_TO_SPIKE_1);
                }
                break;

            case DRIVE_TO_SPIKE_1:
                if (!follower.isBusy()) {
                    follower.followPath(p10);
                    intake.setPower(1.0); // KEEP INTAKE ON for Spike 1 (Last Intake)
                    blocker.setPosition(blockerClosed);
                    setPathState(PathState.DRIVE_TO_SHOOT_5);
                }
                break;

            case DRIVE_TO_SHOOT_5:
                // Kill intake halfway through path for Spike 1 sequence
                if (pathTimer.getElapsedTimeSeconds() > intakeExtraTrafficTime) {
                    intake.setPower(0);
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_5);
                }
                break;

            case SHOOT_5:
                runShootingLogic(PathState.DONE);
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

        PIDFCoefficients pidf = new PIDFCoefficients(85, 0, 0, 15);
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

        // 2. Save the Turret's final raw encoder ticks
        if (turretController != null) {
            PedroPose.saveTurretTicks(turretController.getRawTicks());
        }

        super.stop();
    }
}