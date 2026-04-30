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
public class Ball18Far extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo blocker;

    // External Turret Controller
    private TurretController turretController;
    public double shootVelocity = 1500;

    // --- Configurable Timings & Positions ---
    public double spinUpTime = 0.8;
    public double blockerOpen = 0.3;
    public double blockerClosed = 1.0;
    public double fullDumpDuration = 1.2;  // How long to stay open to let all balls out
    public double blockerMoveDelay = 0.25;

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    private Pose lastPose = new Pose(0,0,0);
    private long lastUpdateTime = 0;
    private Pose currentVelocity = new Pose(0,0,0);

    private boolean isShooting = false;

    public enum PathState {
        INITIAL_SHOOT_SPINUP, INITIAL_SHOOT_ACTION,
        PREP_INTAKE, INTAKE_HUMAN, GO_OUT, GO_BACK_IN,
        STATION_2_SPINUP, STATION_2_SHOOT,
        SPIKE_3,
        STATION_3_SPINUP, STATION_3_SHOOT,
        CHANCE_GRAB_1,
        STATION_4_SPINUP, STATION_4_SHOOT,
        CHANCE_GRAB_2,
        STATION_5_SPINUP, STATION_5_SHOOT,
        CHANCE_GRAB_3,
        STATION_6_SPINUP, STATION_6_SHOOT,
        LEAVE, DONE
    }

    PathState pathState;

    // --- Poses ---
    private final Pose startPose    = new Pose(54.1, 7.2,  Math.toRadians(90));
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

    /**
     * Opens once, lets everything fly out, then closes.
     */
    private void runShootingLogic(PathState nextState) {
        if (!isShooting) {
            blocker.setPosition(blockerOpen);
            intake.setPower(1.0); // Ensure intake is pushing balls into shooter
            isShooting = true;
            kickTimer.resetTimer();
        } else {
            // Once the time is up, close and move on
            if (kickTimer.getElapsedTimeSeconds() > fullDumpDuration) {
                blocker.setPosition(blockerClosed);
                setPathState(nextState);
            }
        }
    }

    public void statePathUpdate() {
        follower.setMaxPower(0.9);

        switch (pathState) {
            case INITIAL_SHOOT_SPINUP:
                if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                    setPathState(PathState.INITIAL_SHOOT_ACTION);
                }
                break;

            case INITIAL_SHOOT_ACTION:
                runShootingLogic(PathState.PREP_INTAKE);
                break;

            case PREP_INTAKE:
                blocker.setPosition(blockerClosed);
                intake.setPower(0); // Stop intake after dumping
                if (pathTimer.getElapsedTimeSeconds() > blockerMoveDelay) {
                    setPathState(PathState.INTAKE_HUMAN);
                }
                break;

            case INTAKE_HUMAN:
                if (!follower.isBusy()) {
                    follower.followPath(p1);
                    intake.setPower(1.0);
                    setPathState(PathState.GO_OUT);
                }
                break;

            case GO_OUT:
                if (!follower.isBusy()) {
                    follower.followPath(p2);
                    setPathState(PathState.GO_BACK_IN);
                }
                break;

            case GO_BACK_IN:
                if (!follower.isBusy()) {
                    follower.followPath(p3);
                    setPathState(PathState.STATION_2_SPINUP);
                }
                break;

            case STATION_2_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p4);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.STATION_2_SHOOT);
                    }
                }
                break;

            case STATION_2_SHOOT:
                runShootingLogic(PathState.SPIKE_3);
                break;

            case SPIKE_3:
                if (!follower.isBusy()) {
                    intake.setPower(0); // Stop intake while traveling
                    follower.followPath(p5);
                    setPathState(PathState.STATION_3_SPINUP);
                }
                break;

            case STATION_3_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p6);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.STATION_3_SHOOT);
                    }
                }
                break;

            case STATION_3_SHOOT:
                runShootingLogic(PathState.CHANCE_GRAB_1);
                break;

            case CHANCE_GRAB_1:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p7);
                    setPathState(PathState.STATION_4_SPINUP);
                }
                break;

            case STATION_4_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p8);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.STATION_4_SHOOT);
                    }
                }
                break;

            case STATION_4_SHOOT:
                runShootingLogic(PathState.CHANCE_GRAB_2);
                break;

            case CHANCE_GRAB_2:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p9);
                    setPathState(PathState.STATION_5_SPINUP);
                }
                break;

            case STATION_5_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p10);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.STATION_5_SHOOT);
                    }
                }
                break;

            case STATION_5_SHOOT:
                runShootingLogic(PathState.CHANCE_GRAB_3);
                break;

            case CHANCE_GRAB_3:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p11);
                    setPathState(PathState.STATION_6_SPINUP);
                }
                break;

            case STATION_6_SPINUP:
                if (!follower.isBusy()) {
                    follower.followPath(p12);
                    if (pathTimer.getElapsedTimeSeconds() > spinUpTime) {
                        setPathState(PathState.STATION_6_SHOOT);
                    }
                }
                break;

            case STATION_6_SHOOT:
                runShootingLogic(PathState.LEAVE);
                break;

            case LEAVE:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p13);
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
        setPathState(PathState.INITIAL_SHOOT_SPINUP);
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
        panelsTelemetry.update(telemetry);
    }
}