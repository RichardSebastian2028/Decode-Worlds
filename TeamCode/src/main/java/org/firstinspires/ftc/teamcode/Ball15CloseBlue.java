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
    public double shootVelocity = 1300;

    // --- Configurable Timings & Positions ---
    public double spinUpTime = 0.8;
    public double blockerOpen = 0.3;
    public double blockerClosed = 1.0;
    public double fullDumpDuration = 1;
    public double blockerMoveDelay = 0.25;

    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    private Pose lastPose = new Pose(0,0,0);
    private long lastUpdateTime = 0;
    private Pose currentVelocity = new Pose(0,0,0);

    private boolean isShooting = false;

    public enum PathState {
        WAIT_FOR_HOMING,
        INITIAL_SPINUP, SHOOT_PRELOAD,
        PREP_INTAKE_1, INTAKE_SPIKE_2,
        OPEN_GATE,
        STATION_2_SPINUP, SHOOT_2,
        PREP_INTAKE_2, INTAKE_GATE_1,
        STATION_3_SPINUP, SHOOT_3,
        PREP_INTAKE_3, INTAKE_GATE_2,
        STATION_4_SPINUP, SHOOT_4,
        PREP_INTAKE_4, INTAKE_SPIKE_1,
        STATION_5_SPINUP, SHOOT_5,
        DONE
    }

    PathState pathState;

    // --- Poses from JSON ---
    private final Pose startPose      = new Pose(23.4, 125.2, Math.toRadians(135));
    private final Pose shootPreload   = new Pose(58.0, 82.7,  Math.toRadians(180));
    private final Pose controlshootPreloadintakeSpike2 = new Pose(66.1, 56.3, Math.toRadians(180));
    private final Pose intakeSpike2   = new Pose(14.4, 58.9,  Math.toRadians(180));
    private final Pose controlintakeSpike2openGatePose = new Pose(35.9, 64.6, Math.toRadians(180));
    private final Pose openGatePose   = new Pose(15.1, 68.4,  Math.toRadians(180));
    private final Pose intakeGatePose = new Pose(9.5,  58.5,  Math.toRadians(140));
    private final Pose intakeSpike1   = new Pose(15.2, 82.6,  Math.toRadians(180));
    private final Pose shoot5Pose     = new Pose(56.5, 105.7, Math.toRadians(180));
    private final Pose contolOpenGatePoseShootPreload = new Pose(61.3, 60.8, Math.toRadians(180));
    private final Pose controlShootPreloadIntakeGatePose = new Pose(61.3, 60.8, Math.toRadians(180));
    private final Pose controlIntakeGatePoseShootPreload = new Pose(50.9, 67.0, Math.toRadians(180));


    private PathChain p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;

    public void buildPaths() {
        // Path 1: Start to ShootPreload
        p1 = follower.pathBuilder().addPath(new BezierLine(startPose, shootPreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreload.getHeading()).build();

        // Path 2: ShootPreload to IntakeSpike2 (Curve)
        p2 = follower.pathBuilder().addPath(new BezierCurve(shootPreload, controlshootPreloadintakeSpike2, intakeSpike2))
                .setConstantHeadingInterpolation(intakeSpike2.getHeading()).build();

        // Path 3: IntakeSpike2 to OpenGate (Curve)
        p3 = follower.pathBuilder().addPath(new BezierCurve(intakeSpike2, controlintakeSpike2openGatePose, openGatePose))
                .setConstantHeadingInterpolation(openGatePose.getHeading()).build();

        // Path 4: OpenGate to Shoot2 (Curve)
        p4 = follower.pathBuilder().addPath(new BezierCurve(openGatePose, contolOpenGatePoseShootPreload, shootPreload))
                .setConstantHeadingInterpolation(shootPreload.getHeading()).build();

        // Path 5: Shoot2 to IntakeGate (Curve)
        p5 = follower.pathBuilder().addPath(new BezierCurve(shootPreload, controlShootPreloadIntakeGatePose, intakeGatePose))
                .setLinearHeadingInterpolation(shootPreload.getHeading(), intakeGatePose.getHeading()).build();

        // Path 6: IntakeGate to Shoot3 (Curve)
        p6 = follower.pathBuilder().addPath(new BezierCurve(intakeGatePose, controlIntakeGatePoseShootPreload, shootPreload))
                .setLinearHeadingInterpolation(intakeGatePose.getHeading(), shootPreload.getHeading()).build();

        // Path 7: Shoot3 to IntakeGate2 (Reuse p5 logic/points)
        p7 = follower.pathBuilder().addPath(new BezierCurve(shootPreload,controlShootPreloadIntakeGatePose, intakeGatePose))
                .setLinearHeadingInterpolation(shootPreload.getHeading(), intakeGatePose.getHeading()).build();

        // Path 8: IntakeGate2 to Shoot4 (Reuse p6 logic/points)
        p8 = follower.pathBuilder().addPath(new BezierCurve(intakeGatePose, controlIntakeGatePoseShootPreload, shootPreload))
                .setLinearHeadingInterpolation(intakeGatePose.getHeading(), shootPreload.getHeading()).build();

        // Path 9: Shoot4 to IntakeSpike1
        p9 = follower.pathBuilder().addPath(new BezierLine(shootPreload, intakeSpike1))
                .setConstantHeadingInterpolation(intakeSpike1.getHeading()).build();

        // Path 10: IntakeSpike1 to Shoot5
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
            intake.setPower(1.0);
            isShooting = true;
            kickTimer.resetTimer();
        } else {
            if (kickTimer.getElapsedTimeSeconds() > fullDumpDuration) {
                blocker.setPosition(blockerClosed);
                setPathState(nextState);
            }
        }
    }

    public void statePathUpdate() {
        follower.setMaxPower(0.9);

        switch (pathState) {

            // --- NEW HOMING STATE WITH FAILSAFE ---
            case WAIT_FOR_HOMING:
                // Wait for tracking to start, OR timeout after 3.0 seconds so the auto doesn't die!
                if (turretController.currentState == TurretController.TurretState.TRACKING || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    setPathState(PathState.INITIAL_SPINUP);
                }
                break;

            case INITIAL_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p1);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                }
                break;

            case SHOOT_PRELOAD:
                runShootingLogic(PathState.PREP_INTAKE_1);
                break;

            case PREP_INTAKE_1:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    setPathState(PathState.INTAKE_SPIKE_2);
                }
                break;

            case INTAKE_SPIKE_2:
                if (!follower.isBusy()) {
                    follower.followPath(p2);
                    intake.setPower(1.0);
                    setPathState(PathState.OPEN_GATE);
                }
                break;

            case OPEN_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(p3);
                    setPathState(PathState.STATION_2_SPINUP);
                }
                break;

            case STATION_2_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p4);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.SHOOT_2);
                    }
                }
                break;

            case SHOOT_2:
                runShootingLogic(PathState.PREP_INTAKE_2);
                break;

            case PREP_INTAKE_2:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    setPathState(PathState.INTAKE_GATE_1);
                }
                break;

            case INTAKE_GATE_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(p5);
                    intake.setPower(1.0);
                    setPathState(PathState.STATION_3_SPINUP);
                }
                break;

            case STATION_3_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p6);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.SHOOT_3);
                    }
                }
                break;

            case SHOOT_3:
                runShootingLogic(PathState.PREP_INTAKE_3);
                break;

            case PREP_INTAKE_3:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    setPathState(PathState.INTAKE_GATE_2);
                }
                break;

            case INTAKE_GATE_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(p7);
                    intake.setPower(1.0);
                    setPathState(PathState.STATION_4_SPINUP);
                }
                break;

            case STATION_4_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p8);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.SHOOT_4);
                    }
                }
                break;

            case SHOOT_4:
                runShootingLogic(PathState.PREP_INTAKE_4);
                break;

            case PREP_INTAKE_4:
                blocker.setPosition(blockerClosed);
                intake.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    setPathState(PathState.INTAKE_SPIKE_1);
                }
                break;

            case INTAKE_SPIKE_1:
                if (!follower.isBusy()) {
                    follower.followPath(p9);
                    intake.setPower(1.0);
                    setPathState(PathState.STATION_5_SPINUP);
                }
                break;

            case STATION_5_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p10);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.SHOOT_5);
                    }
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

        PIDFCoefficients pidf = new PIDFCoefficients(90, 0, 0, 16);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        blocker.setPosition(blockerClosed);
        buildPaths();

        // Start the state machine in the new HOMING wait state
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

        // This method automatically handles homing first, then smoothly transitions to tracking
        turretController.aimAtGoalWithPrediction(currentPose, currentVelocity);

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Turret State", turretController.currentState);
        panelsTelemetry.update(telemetry);
    }
}